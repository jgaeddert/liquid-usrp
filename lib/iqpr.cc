//
// liquid packet radio
//

#include <iostream>
#include <complex>
#include <math.h>
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
    flexframesyncprops_s fsprops;   // frame synchronizer properties

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
    unsigned char header_tx[8];     // transmit header
    unsigned char header_rx[8];     // receive header

    // buffers
    unsigned int payload_len;       // decoded message length (bytes)
    unsigned int packet_len;        // encoded message length (bytes)
    unsigned int frame_len;         // frame length (complex samples)
    std::complex<float> * frame;    // size: 1 x frame_len
    std::complex<float> * mf_buffer[256];
    std::complex<float> data_tx[512];

    // status variables
    int verbose;
    unsigned int num_packets_received;
    unsigned int num_valid_headers_received;
    unsigned int num_valid_packets_received;
    unsigned int num_bytes_received;

    // threads
    pthread_t tx_thread;
    pthread_t rx_thread;
};

iqpr iqpr_create()
{
    iqpr q = (iqpr) malloc(sizeof(struct iqpr_s));

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
    flexframesyncprops_init_default(&q->fsprops);
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

    // clear status
    q->verbose = 1;

    // start tx/rx threads
    iqpr_start_threads(q);

    // return object
    return q;
}

void iqpr_destroy(iqpr _q)
{
    // stop tx/rx threads
    iqpr_stop_threads(_q);

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
    unsigned int packet_id = (_rx_header[0] << 8 | _rx_header[1]);
    if (q->verbose) printf("packet id: %6u\n", packet_id);
    unsigned int payload_len = (_rx_header[2] << 8 | _rx_header[3]);
    fec_scheme fec0 = (fec_scheme)(_rx_header[4]);
    fec_scheme fec1 = (fec_scheme)(_rx_header[5]);

    /*
    // TODO: validate fec0,fec1 before indexing fec_scheme_str
    printf("    payload len : %u\n", payload_len);
    printf("    fec0        : %s\n", fec_scheme_str[fec0]);
    printf("    fec1        : %s\n", fec_scheme_str[fec1]);
    */

    q->p_dec = packetizer_recreate(q->p_dec, payload_len, fec0, fec1);

    // decode packet
    unsigned char msg_dec[payload_len];
    bool crc_pass = packetizer_decode(q->p_dec, _rx_payload, msg_dec);
    if (crc_pass) {
        q->num_valid_packets_received++;
        q->num_bytes_received += payload_len;
    } else {
        if (q->verbose) printf("  <<< payload crc fail >>>\n");
    }

    /*
    packetizer_print(q->p_dec);
    printf("payload len: %u\n", _rx_payload_len);
    unsigned int i;
    for (i=0; i<_rx_payload_len; i++)
        printf("%.2x ", _rx_payload[i]);
    printf("\n");

    for (i=0; i<payload_len; i++)
        printf("%.2x ", msg_dec[i]);
    printf("\n");
    */

    return 0;
}

// start threads
void iqpr_start_threads(iqpr _q)
{
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
    // wait for data

    printf("iqpr tx process complete.\n");
    pthread_exit(NULL);
}

void * iqpr_rx_process(void * _q)
{
    // continuously read data, blocking on tx_mutex

    printf("iqpr rx process complete.\n");
    pthread_exit(NULL);
}


