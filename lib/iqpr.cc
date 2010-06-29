//
// liquid packet radio
//

#include <iostream>
#include <complex>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <liquid/liquid.h>

#include "iqpr.h"
#include "usrp_io.h"

#define USRP_CHANNEL        (0)

// packet header structure
//
//  id      num bytes   description
//  -----------------------------
//  plen    2           packet length
//  fec0    1           fec scheme (inner)
//  fec1    1           fec scheme (outer)
//

struct iqpr_s {
    unsigned int node_id;           // node identification
    unsigned int net_id;            // network identification

    usrp_io * uio;                  // usrp input/output device

    // framing objects
    flexframegen fg;                // frame generator
    flexframegenprops_s fgprops;    // frame generator properties
    flexframesync fs;               // frame synchronizer
    framesyncprops_s fsprops;       // frame synchronizer properties

    // filtering objects
    interp_crcf  mf_interp;         // matched filter interpolator
    resamp2_crcf interp;            // half-band interpolator
    resamp2_crcf decim;             // half-band decimator

    // packetizer objects
    packetizer p_enc;               // packet encoder
    packetizer p_dec;               // packet decoder

    // certain properties
    float gain_tx;                  // transmit gain

    // MAC layer
    int tx_mutex;                   // transmit mutex lock for...
    unsigned char header_tx[9];     // transmit header
    unsigned char header_rx[9];     // receive header

    // buffers
    unsigned int payload_len;       // decoded message length (bytes)
    unsigned int packet_len;        // encoded message length (bytes)
    unsigned int frame_len;         // frame length (complex samples)
    std::complex<float> * frame;    // transmitter frame [size: 1 x frame_len]
    std::complex<float> mf_buffer[256];
    std::complex<float> data_tx[512];
    unsigned int data_rx_numalloc;  // number of bytes allocated to data_rx
    unsigned int data_rx_len;       // received data buffer length
    unsigned char * data_rx;        // received data buffer

    // status variables
    int verbose;
    unsigned int num_packets_received;
    unsigned int num_valid_headers_received;
    unsigned int num_valid_packets_received;
    unsigned int num_bytes_received;
    unsigned int num_collisions;

    // threads
    pthread_t tx_thread;
    pthread_t rx_thread;

    // mutexes, conditional variables
    pthread_mutex_t usrp_mutex;     // usrp mutex
    int tx_hold;                    // hold transmitter flag (csma)
    int tx_waiting;                 // transmitter waiting flag
    pthread_cond_t tx_data_ready;   // transmitter data condition
    pthread_cond_t tx_hold_ready;   // transmitter hold condition
};

iqpr iqpr_create(unsigned int _node_id)
{
    iqpr q = (iqpr) malloc(sizeof(struct iqpr_s));

    // set properties
    q->node_id = _node_id;

    // create usrp_io
    q->uio = new usrp_io();
    q->uio->set_rx_freq(USRP_CHANNEL, 462e6f);
    q->uio->set_tx_freq(USRP_CHANNEL, 462e6f);
    q->uio->set_rx_samplerate(2.0f*62.5e3f);
    q->uio->set_tx_samplerate(2.0f*62.5e3f);
    q->uio->enable_auto_tx(USRP_CHANNEL);

    // buffers, etc.
    q->payload_len = 64;
    q->packet_len = packetizer_get_packet_length(q->payload_len,
                                                 FEC_NONE,
                                                 FEC_NONE);

    // create frame generator
    q->fgprops.rampup_len = 16;
    q->fgprops.phasing_len = 64;
    q->fgprops.payload_len = 0;
    q->fgprops.mod_scheme = MOD_QPSK;
    q->fgprops.mod_bps = 2;
    q->fgprops.rampdn_len = 16;
    q->fg = flexframegen_create(&q->fgprops);

    // create frame synchronizer
    framesyncprops_init_default(&q->fsprops);
    q->fs = flexframesync_create(&q->fsprops, iqpr_callback, (void*)q);

    // filtering objects
    q->mf_interp = interp_crcf_create_rrc(2,3,0.7f,0.0f);
    q->interp = resamp2_crcf_create(37,0.0f,60.0f);
    q->decim  = resamp2_crcf_create(37,0.0f,60.0f);

    // packetizer objects
    q->p_enc = packetizer_create(0,FEC_NONE,FEC_NONE);
    q->p_dec = packetizer_create(0,FEC_NONE,FEC_NONE);

    // allocate memory for arrays
    q->frame_len = flexframegen_getframelen(q->fg);
    q->frame = (std::complex<float>*) malloc( q->frame_len * sizeof(std::complex<float>) );
    q->data_rx_numalloc = 64;
    q->data_rx_len = 0;
    q->data_rx = (unsigned char*) malloc( q->data_rx_numalloc * sizeof(unsigned char) );

    // clear status
    q->verbose = 1;

    // initialize mutexes, conditional variables
    pthread_mutex_init(&q->usrp_mutex, NULL);
    pthread_cond_init(&q->tx_data_ready, NULL);
    pthread_cond_init(&q->tx_hold_ready, NULL);
    q->tx_hold = 0;
    q->tx_waiting = 0;

    // start tx/rx threads
    iqpr_start_threads(q);

    // return object
    return q;
}

void iqpr_destroy(iqpr _q)
{
    // stop tx/rx threads
    iqpr_stop_threads(_q);

    // destroy mutexes, conditional variables
    pthread_mutex_destroy(&_q->usrp_mutex);
    pthread_cond_destroy(&_q->tx_data_ready);
    pthread_cond_destroy(&_q->tx_hold_ready);

    // destroy usrp_io object
    delete _q->uio;

    // destroy framing objects
    flexframegen_destroy(_q->fg);
    flexframesync_destroy(_q->fs);

    // destroy filter objects
    interp_crcf_destroy(_q->mf_interp);
    resamp2_crcf_destroy(_q->interp);
    resamp2_crcf_destroy(_q->decim);

    // destroy packetizer objects
    packetizer_destroy(_q->p_enc);
    packetizer_destroy(_q->p_dec);

    // free allocated memory
    free(_q->frame);

    // free main object
    free(_q);
}

void iqpr_print(iqpr _q)
{
    printf("iqpr:\n");
}

// 
// internal methods
//

// callback
int iqpr_callback(unsigned char * _rx_header,
                  int _rx_header_valid,
                  unsigned char * _rx_payload,
                  unsigned int _rx_payload_len,
                  framesyncstats_s _stats,
                  void * _userdata)
{
    iqpr q = (iqpr) _userdata;

    q->num_packets_received++;
    if (q->verbose) printf("********* iqpr callback invoked, ");

    if ( !_rx_header_valid ) {
        if (q->verbose) printf("header crc : FAIL\n");
        return 0;
    }
    q->num_valid_headers_received++;

    // decode header
    struct iqpr_header_s header;
    iqpr_header_decode(_rx_header, &header);

    if (q->verbose) printf("packet id: %6u\n", header.packet_id);

    q->p_dec = packetizer_recreate(q->p_dec,
                                   header.payload_len,
                                   header.fec0,
                                   header.fec1);

    // decode packet, reallocating buffer if necessary
    if (header.payload_len > q->data_rx_numalloc) {
        printf("iqpr, reallocating rx data length to %u bytes\n", header.payload_len);
        q->data_rx_numalloc = header.payload_len;
        q->data_rx = (unsigned char*) realloc(q->data_rx, q->data_rx_numalloc*sizeof(unsigned char));
        if (q->data_rx == NULL) {
            fprintf(stderr,"error, could not allocate memory for received data array\n");
            exit(1);
        }
    }
    q->data_rx_len = header.payload_len;
    bool crc_pass = packetizer_decode(q->p_dec, _rx_payload, q->data_rx);
    if (crc_pass) {
        q->num_valid_packets_received++;
        q->num_bytes_received += header.payload_len;

        // TODO : check packet type, send request for immediate ACK if necessary
    } else {
        if (q->verbose) printf("  <<< payload crc fail >>>\n");
    }

    return 0;
}

void iqpr_callback_csma_lock(void * _userdata)
{
    iqpr q = (iqpr) _userdata;

    // set tx hold flag
    q->tx_hold = 1;
}

void iqpr_callback_csma_unlock(void * _userdata)
{
    iqpr q = (iqpr) _userdata;

    // release tx hold flag
    q->tx_hold = 0;

    // if transmitter is waiting for data, signal condition
    if (q->tx_waiting) {
        pthread_mutex_lock(&q->usrp_mutex);

        pthread_cond_signal(&q->tx_hold_ready);

        pthread_mutex_unlock(&q->usrp_mutex);
    }
}


// start threads
void iqpr_start_threads(iqpr _q)
{
    // start usrp_io
    _q->uio->start_rx(USRP_CHANNEL);
    _q->uio->start_tx(USRP_CHANNEL);

    // set thread attributes
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    // create tx/rx threads
    pthread_create(&_q->tx_thread, &attr, iqpr_tx_process, (void*)_q);
    pthread_create(&_q->rx_thread, &attr, iqpr_rx_process, (void*)_q);
}

// stop threads
void iqpr_stop_threads(iqpr _q)
{
    void * status;
    pthread_join(_q->tx_thread, &status);
    pthread_join(_q->rx_thread, &status);
}

void * iqpr_tx_process(void * _q)
{
    iqpr q = (iqpr) _q;

    // TODO : wait for data to become avilable
    unsigned int i;
    for (i=0; i<100; i++) {
        usleep(10000);
        // wait for data
        // pthread_mutex_lock(&q->usrp_mutex);
        //
        // // this automatically unlocks usrp_mutex
        // pthread_cond_wait(&q->tx_data_ready,
        //                   &q->usrp_mutex);

        // received data transmit request!

        // assemble frame...

        // check if tx hold is on
        if (q->tx_hold) {
            // set flag to tell csma that transmitter is waiting
            q->tx_waiting = 1;

            // this atomically unlocks usrp_mutex and waits for
            // pthread_cond_signal(&q->tx_hold_ready)
            pthread_cond_wait(&q->tx_hold_ready,
                              &q->usrp_mutex);

            // clear waiting flag
            q->tx_waiting = 0;
        }

        // transmit frame

        // pthread_mutex_unlock(&q->usrp_mutex);
    }

    printf("iqpr tx process complete.\n");
    pthread_exit(NULL);
}

void * iqpr_rx_process(void * _q)
{
    iqpr q = (iqpr) _q;

    // ports, buffers, etc.
    gport port_rx = q->uio->get_rx_port(USRP_CHANNEL);
    std::complex<float> data_rx[512];
    std::complex<float> decim_out[256];

    // continuously read data, blocking on tx_mutex
    unsigned int i,n;
    unsigned int num_blocks = 1000;
    for (i=0; i<num_blocks; i++) {
        // grab data from port
        gport_consume(port_rx, (void*)data_rx, 512);

        // run decimator
        for (n=0; n<256; n++) {
            resamp2_crcf_decim_execute(q->decim, &data_rx[2*n], &decim_out[n]);
        }

        // run through frame synchronizer
        flexframesync_execute(q->fs, decim_out, 256);
    }

    printf("iqpr rx process complete.\n");
    pthread_exit(NULL);
}

void iqpr_header_print(struct iqpr_header_s _header)
{
    printf("iqpr header:\n");
    printf("    packet id       :   %u\n", _header.packet_id);
    printf("    payload length  :   %u\n", _header.payload_len);
    printf("    fec (inner)     :   %s\n", fec_scheme_str[_header.fec0]);
    printf("    fec (outer)     :   %s\n", fec_scheme_str[_header.fec1]);
    printf("    source node id  :   %u\n", _header.node_id_src);
    printf("    dest. node id   :   %u\n", _header.node_id_dst);
    printf("    packet type     :   %u\n", _header.packet_type);
}

void iqpr_header_encode(struct iqpr_header_s _header,
                        unsigned char * _header_data)
{
    // encode packet id
    _header_data[0] = (_header.packet_id >> 8) & 0x00ff;
    _header_data[1] = (_header.packet_id     ) & 0x00ff;

    // encode payload length
    _header_data[2] = (_header.payload_len >> 8) & 0x00ff;
    _header_data[3] = (_header.payload_len     ) & 0x00ff;

    // encode fec schemes
    _header_data[4] = (unsigned char) _header.fec0;
    _header_data[5] = (unsigned char) _header.fec1;

    // encode source/destination node IDs
    _header_data[6] = (unsigned char) _header.node_id_src;
    _header_data[7] = (unsigned char) _header.node_id_dst;

    // encode packet type
    _header_data[8] = _header.packet_type & 0x00ff;
}

void iqpr_header_decode(unsigned char * _header_data,
                        struct iqpr_header_s * _header)
{
    // decode packet id
    _header->packet_id = (_header_data[0] << 8) | _header_data[1];

    // decode packet length
    _header->payload_len = (_header_data[2] << 8) | _header_data[3];

    // decode fec schemes
    _header->fec0 = (fec_scheme)(_header_data[4]);
    _header->fec1 = (fec_scheme)(_header_data[5]);

    // decode source/destination node IDs
    _header->node_id_src = _header_data[6];
    _header->node_id_dst = _header_data[7];

    // decode packet type
    _header->packet_type = _header_data[8];
}


