/*
 * Copyright (c) 2007, 2008, 2009, 2010 Joseph Gaeddert
 * Copyright (c) 2007, 2008, 2009, 2010 Virginia Polytechnic
 *                                      Institute & State University
 *
 * This file is part of liquid.
 *
 * liquid is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * liquid is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with liquid.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// ping.cc
//
// ping basic data packets back and forth
//

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <getopt.h>
#include <pthread.h>
#include <sys/time.h>
#include <complex>
#include <liquid/liquid.h>
#include "usrp_io.h"

#define USRP_CHANNEL    (0)

#define NODE_MASTER     (0)
#define NODE_SLAVE      (1)

void * tx_handler( void * _port );
void * rx_handler( void * _port );
void * pm_handler( void * _port );
 
void usage() {
    printf("ping usage:\n");
    printf("  u,h   :   usage/help\n");
    printf("  f     :   frequency [Hz], default: 462 MHz\n");
    printf("  b     :   bandwidth [Hz], default: 100 kHz\n");
    printf("  m/s   :   designate node as master/slave, default: slave\n");
}

// ping data structure
typedef struct {
    gport port_tx;                  // transmit port
    gport port_rx;                  // receive port

    // common
    int continue_running;           // continue running transceiver flag
    int node_type;                  // master/slave
    unsigned int payload_len;       // payload length (raw data)
    unsigned int packet_len;        // packet length (encoded data)
    unsigned int pid;               // packet id

    // receiver
    pthread_mutex_t rx_data_mutex;
    unsigned int ack_timeout_us;    // time to wait for acknowledgement (us)
    packetizer p_dec;               // packet decoder

    // transmitter
    pthread_mutex_t tx_data_mutex;
    pthread_cond_t  tx_data_ready;
    unsigned char * tx_data;
    unsigned int packet_id_tx;
    packetizer p_enc;               // packet encoder

    // packet manager
    unsigned int num_timeouts;      // running timeout counter
} pingdata;

static int ping_callback(unsigned char * _rx_header,
                         int _rx_header_valid,
                         unsigned char * _rx_payload,
                         unsigned int _rx_payload_len,
                         framesyncstats_s _stats,
                         void * _userdata);
void pingdata_init(pingdata * _q);
void pingdata_destroy(pingdata * _q);

// initialize timespec given microseconds
void ping_init_timespec(struct timespec * _ts,
                        unsigned int _usec);

int main (int argc, char **argv) {
    // options
    float frequency = 462e6f;
    float symbolrate = 100e3f;

    // initialize pingdata structure
    pingdata q;
    pingdata_init(&q);

    //
    int d;
    while ((d = getopt(argc,argv,"uhf:b:ms")) != EOF) {
        switch (d) {
        case 'u':
        case 'h': usage();                      return 0;
        case 'f': frequency = atof(optarg);     break;
        case 'b': symbolrate = atof(optarg);    break;
        case 'm': q.node_type = NODE_MASTER;    break;
        case 's': q.node_type = NODE_SLAVE;     break;
        default:
            fprintf(stderr,"error: %s, unsupported option\n", argv[0]);
            exit(1);
        }
    }

    // create usrp object
    usrp_io * usrp = new usrp_io();

    // set properties
    usrp->set_tx_freq(USRP_CHANNEL, frequency);
    usrp->set_tx_samplerate(2*symbolrate);
    usrp->set_rx_freq(USRP_CHANNEL, frequency);
    usrp->set_rx_samplerate(2*symbolrate);
    usrp->enable_auto_tx(USRP_CHANNEL);

    // initialize ports
    q.port_tx = usrp->get_tx_port(USRP_CHANNEL);
    q.port_rx = usrp->get_rx_port(USRP_CHANNEL);

    // threads
    pthread_t tx_thread;
    pthread_t rx_thread;
    pthread_t pm_thread;
    pthread_attr_t thread_attr;
    void * status;
    
    // set thread attributes
    pthread_attr_init(&thread_attr);
    pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_JOINABLE);

    // attributes object no longer needed
    pthread_attr_destroy(&thread_attr);

    printf("waiting to start threads...\n");
    usleep(1000000);

    // create threads
    pthread_create(&tx_thread, &thread_attr, &tx_handler, (void*) &q);
    pthread_create(&rx_thread, &thread_attr, &rx_handler, (void*) &q);
    pthread_create(&pm_thread, &thread_attr, &pm_handler, (void*) &q);

    // start data transfer
    usrp->start_tx(USRP_CHANNEL);
    usrp->start_rx(USRP_CHANNEL);

    printf("waiting for threads to exit...\n");

    // join threads
    pthread_join(tx_thread, &status);
    pthread_join(rx_thread, &status);
    pthread_join(pm_thread, &status);

    // stop
    usrp->stop_rx(USRP_CHANNEL);
    usrp->stop_tx(USRP_CHANNEL);

    printf("main process complete\n");

    // delete usrp object
    delete usrp;

    // destroy main data object
    pingdata_destroy(&q);

    return 0;
}

void * tx_handler ( void * _userdata )
{
    pingdata * q = (pingdata*) _userdata;

    // create flexframe generator
    flexframegenprops_s fgprops;
    fgprops.rampup_len = 64;
    fgprops.phasing_len = 64;
    fgprops.payload_len = q->packet_len;    // NOTE : payload_len for frame is packet_len
    fgprops.mod_scheme = MOD_QPSK;
    fgprops.mod_bps = 2;
    fgprops.rampdn_len = 64;

    flexframegen fg = flexframegen_create(&fgprops);
    flexframegen_print(fg);

    // interpolator options
    unsigned int m=3;
    float beta=0.7f;
    interp_crcf interp = interp_crcf_create_rrc(2,m,beta,0);

    unsigned int i;
    float g = 0.5f; // transmit gain

    // allocate memory for buffers
    unsigned char header[14];
    unsigned char payload[q->payload_len];  // raw data
    unsigned char packet[q->packet_len];    // encoded message length

    unsigned int frame_len = flexframegen_getframelen(fg);
    std::complex<float> frame[frame_len];

    std::complex<float> mfbuffer[2*frame_len];

    printf("tx thread running...\n");

    while (q->continue_running)
    {
#if 0
        // wait for signal condition
        pthread_mutex_lock(&q->tx_data_mutex);
        pthread_cond_wait(&q->tx_data_ready,
                          &q->tx_data_mutex);
#endif

        // check if we have received kill signal
        if (!q->continue_running)
            break;

        printf("[tx] sending packet %u...\n", q->pid);

        // generate frame data
        for (i=0; i<q->payload_len; i++)
            payload[i] = rand() % 256;

        // encode packet
        packetizer_encode(q->p_enc, payload, packet);

        // prepare header
        header[0] = (q->pid >> 8) & 0xff;
        header[1] = (q->pid     ) & 0xff;
        header[2] = (q->payload_len >> 8) & 0xff;
        header[3] = (q->payload_len     ) & 0xff;
        header[4] = (unsigned char)(FEC_NONE);
        header[5] = (unsigned char)(FEC_NONE);

        // generate frame
        flexframegen_execute(fg, header, packet, frame);

#if 0
        // release tx mutex
        pthread_mutex_unlock(&q->tx_data_mutex);
#else
        q->pid = (q->pid+1) & 0xffff;
#endif

        // run interpolator
        for (i=0; i<frame_len; i++) {
            interp_crcf_execute(interp, frame[i]*g, &mfbuffer[2*i]);
        }

        // produce data in buffer
        gport_produce(q->port_tx, (void*)mfbuffer, 2*frame_len);

        // flush interpolator with zeros
        for (i=0; i<32; i++) {
            interp_crcf_execute(interp, 0, &mfbuffer[2*i]);
        }

        gport_produce(q->port_tx, (void*)mfbuffer, 64);

        usleep(1000000);
    }

    // clean up allocated memory objects
    flexframegen_destroy(fg);
    interp_crcf_destroy(interp);
   
    printf("tx_handler finished.\n");
    pthread_exit(0); // exit thread
}


void * rx_handler ( void * _userdata )
{
    pingdata * q = (pingdata*) _userdata;

    // create flexframe synchronizer
    framesyncprops_s fsprops;
    framesyncprops_init_default(&fsprops);
    fsprops.squelch_threshold = -30.0f;
    flexframesync fs = flexframesync_create(&fsprops, ping_callback, (void*)q);

    // allocate memory for buffers
    std::complex<float> data_rx[512];

    // read samples from buffer, run through frame synchronizer
    while (q->continue_running) {
        // TODO : use mutex lock ?

        // grab data from port
        gport_consume(q->port_rx,(void*)data_rx,512);

        // run through frame synchronizer
        flexframesync_execute(fs, data_rx, 512);
    }

    // clean up allocated memory
    flexframesync_destroy(fs);

    printf("rx_handler finished.\n");
    pthread_exit(0); // exit thread
}


void * pm_handler ( void * _userdata )
{
    pingdata * q = (pingdata*) _userdata;

    unsigned int i;
    for (i=0; i<10; i++) {
#if 0
        // send data packet
        pthread_mutex_lock(&q->tx_data_mutex);

        // prepare header...

        printf("[pm] sending packet %u...\n", q->pid);

        pthread_mutex_unlock(&q->tx_data_mutex);
        pthread_cond_signal(&q->tx_data_ready);
#endif

        usleep(1000000);
    }

    // signal other threads to stop
    q->continue_running = 0;

    printf("pm_handler finished.\n");
    pthread_exit(0); // exit thread
}


// 
// pingdata internal methods
//

static int ping_callback(unsigned char * _rx_header,
                         int _rx_header_valid,
                         unsigned char * _rx_payload,
                         unsigned int _rx_payload_len,
                         framesyncstats_s _stats,
                         void * _userdata)
{
    pingdata * q = (pingdata*) _userdata;

    int verbose = 1;
    if (verbose) {
        printf("********* callback invoked, ");
        printf("SNR=%5.1fdB, ", _stats.SNR);
        printf("rssi=%5.1fdB, ", _stats.rssi);
    }

    if ( !_rx_header_valid ) {
        if (verbose) printf("header crc : FAIL\n");
        return 0;
    }
    unsigned int packet_id = (_rx_header[0] << 8 | _rx_header[1]);
    if (verbose) printf("packet id: %6u\n", packet_id);
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
    } else {
        if (verbose) printf("  <<< payload crc fail >>>\n");
    }

    return 0;
}


void pingdata_init(pingdata * _q)
{
    _q->continue_running = 1;
    _q->node_type = NODE_MASTER;
    _q->payload_len = 1024;
    _q->packet_len = packetizer_get_packet_length(_q->payload_len, FEC_NONE, FEC_NONE);
    _q->pid = 0;

    // create packetizers
    _q->p_enc = packetizer_create(_q->payload_len, FEC_NONE, FEC_NONE);
    _q->p_dec = packetizer_create(_q->payload_len, FEC_NONE, FEC_NONE);

    // create mutexes, conditions
    pthread_mutex_init(&_q->tx_data_mutex, NULL);
    pthread_mutex_init(&_q->rx_data_mutex, NULL);
    pthread_cond_init(&_q->tx_data_ready, NULL);
}

void pingdata_destroy(pingdata * _q)
{
    // destroy packetizers
    packetizer_destroy(_q->p_enc);
    packetizer_destroy(_q->p_dec);

    // destroy mutexes, conditions
    pthread_mutex_destroy(&_q->tx_data_mutex);
    pthread_mutex_destroy(&_q->rx_data_mutex);
    pthread_cond_destroy(&_q->tx_data_ready);
}

//
// other useful methods
//

// initialize timespec given microseconds
void ping_init_timespec(struct timespec * _ts,
                        unsigned int _usec)
{
    struct timeval tp;
    int rc = gettimeofday(&tp,NULL);

    if (rc == -1) {
        fprintf(stderr,"error: ping_init_timespec(), failed to get time of day\n");
        exit(1);
    }

    _ts->tv_sec = tp.tv_sec;
    _ts->tv_nsec = (tp.tv_usec + 1000*_usec) * 1000;

    while (_ts->tv_nsec > 1000000000) {
        _ts->tv_nsec -= 1000000000;
        _ts->tv_sec += 1;
    }
}

