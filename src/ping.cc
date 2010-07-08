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
    unsigned int payload_len;       // payload length
    unsigned int pid;               // packet id

    // receiver
    unsigned int ack_timeout_us;    // time to wait for acknowledgement (us)

    // transmitter
    pthread_mutex_t tx_data_mutex;
    pthread_cond_t  tx_data_ready;
    unsigned char * tx_data;
    unsigned int packet_id_tx;

    // packet manager
    unsigned int num_timeouts;      // running timeout counter
} pingdata;

void pingdata_init(pingdata * _q);

int main (int argc, char **argv) {
    // options
    float frequency = 462e6f;
    float samplerate = 100e3f;

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
        case 'b': samplerate = atof(optarg);    break;
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
    usrp->set_tx_samplerate(samplerate);
    usrp->set_rx_freq(USRP_CHANNEL, frequency);
    usrp->set_rx_samplerate(samplerate);
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
}

void * tx_handler ( void * _userdata )
{
    pingdata * q = (pingdata*) _userdata;

    // create flexframe generator
    flexframegenprops_s fgprops;
    fgprops.rampup_len = 64;
    fgprops.phasing_len = 64;
    fgprops.payload_len = q->payload_len;
    fgprops.mod_scheme = MOD_QPSK;
    fgprops.mod_bps = 2;
    fgprops.rampdn_len = 64;

    flexframegen fg = flexframegen_create(&fgprops);
    flexframegen_print(fg);

    // interpolator options
    unsigned int m=4;
    float beta=0.3f;
    interp_crcf interp = interp_crcf_create_rrc(2,m,beta,0);

    unsigned int i;
    float g = 0.5f; // transmit gain

    // allocate memory for buffers
    unsigned char header[14];
    unsigned char payload[q->payload_len];

    unsigned int frame_len = flexframegen_getframelen(fg);
    std::complex<float> frame[frame_len];

    std::complex<float> mfbuffer[2*frame_len];

    printf("tx thread running...\n");

    while (q->continue_running)
    {
        // TODO wait for signal...

        // prepare header
        header[0] = (q->pid >> 8) & 0xff;
        header[1] = (q->pid     ) & 0xff;
        header[2] = (q->payload_len >> 8) & 0xff;
        header[3] = (q->payload_len >> 8) & 0xff;
        header[4] = (unsigned char)(FEC_NONE);
        header[5] = (unsigned char)(FEC_NONE);

        // generate frame data
        for (i=0; i<q->payload_len; i++)
            frame[i] = rand() & 0xff;

        // generate frame
        flexframegen_execute(fg, header, payload, frame);

        // run interpolator
        for (i=0; i<frame_len; i++) {
            interp_crcf_execute(interp, frame[i]*g, &mfbuffer[2*i]);
        }

        // produce data in buffer
        gport_produce(q->port_tx, (void*)mfbuffer, 2*frame_len);
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

    std::complex<float> data_rx[512];

    while (q->continue_running) {
        gport_consume(q->port_rx,(void*)data_rx,512);
        
    }

    printf("rx_handler finished.\n");
    pthread_exit(0); // exit thread
}


void * pm_handler ( void * _userdata )
{
    pingdata * q = (pingdata*) _userdata;

    usleep(10000000);

    // signal other threads to stop
    q->continue_running = 0;

    printf("pm_handler finished.\n");
    pthread_exit(0); // exit thread
}


// 
// pingdata internal methods
//

void pingdata_init(pingdata * _q)
{
    _q->continue_running = 1;
    _q->node_type = NODE_MASTER;
    _q->payload_len = 1024;
    _q->pid = 0;
}

