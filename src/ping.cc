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
    unsigned int ack_timeout_us;    // time to wait for acknowledgement (us)
    packetizer p_dec;               // packet decoder
    unsigned char * rx_data;
    unsigned int rx_data_len;
    unsigned char rx_header[14];
    framesyncprops_s fsprops;
    flexframesync fs;
    std::complex<float> data_rx[512];   // rx data buffer

    // transmitter
    packetizer p_enc;               // packet encoder
    unsigned char * tx_data;
    unsigned int tx_data_len;
    unsigned char tx_header[14];
    unsigned int packet_id_tx;
    flexframegenprops_s fgprops;
    flexframegen fg;
    interp_crcf interp;

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
void pingdata_txpacket(pingdata * _q);
void pingdata_rxpacket(pingdata * _q);

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

    usleep(1000000);

    // start data transfer
    usrp->start_tx(USRP_CHANNEL);
    usrp->start_rx(USRP_CHANNEL);

    unsigned int i;
#if 0
    for (i=0; i<(1<<15); i++)
        pingdata_txpacket(&q);
#else
    for (i=0; i<(1<<15); i++)
        pingdata_rxpacket(&q);
#endif

    // stop data transfer
    usrp->stop_rx(USRP_CHANNEL);
    usrp->stop_tx(USRP_CHANNEL);

    printf("main process complete\n");

    // delete usrp object
    delete usrp;

    // destroy main data object
    pingdata_destroy(&q);

    return 0;
}

void pingdata_txpacket(pingdata * _q)
{
    unsigned int i;
    float g = 0.5f; // transmit gain

    // allocate memory for buffers
    unsigned char payload[_q->payload_len];  // raw data
    unsigned char packet[_q->packet_len];    // encoded message length

    unsigned int frame_len = flexframegen_getframelen(_q->fg);
    std::complex<float> frame[frame_len];
    std::complex<float> mfbuffer[2*frame_len];

    printf("[tx] sending packet %u...\n", _q->pid);

    // generate frame data
    for (i=0; i<_q->payload_len; i++)
        payload[i] = rand() % 256;

    // encode packet
    packetizer_encode(_q->p_enc, payload, packet);

    // prepare header
    _q->tx_header[0] = (_q->pid >> 8) & 0xff;
    _q->tx_header[1] = (_q->pid     ) & 0xff;
    _q->tx_header[2] = (_q->payload_len >> 8) & 0xff;
    _q->tx_header[3] = (_q->payload_len     ) & 0xff;
    _q->tx_header[4] = (unsigned char)(FEC_NONE);
    _q->tx_header[5] = (unsigned char)(FEC_NONE);

    // generate frame
    flexframegen_execute(_q->fg, _q->tx_header, packet, frame);

    // run interpolator
    for (i=0; i<frame_len; i++) {
        interp_crcf_execute(_q->interp, frame[i]*g, &mfbuffer[2*i]);
    }

    // produce data in buffer
    gport_produce(_q->port_tx, (void*)mfbuffer, 2*frame_len);

    // flush interpolator with zeros
    for (i=0; i<256; i++) {
        interp_crcf_execute(_q->interp, 0, &mfbuffer[2*i]);
    }

    gport_produce(_q->port_tx, (void*)mfbuffer, 512);

    _q->pid++;
}


void pingdata_rxpacket(pingdata * _q)
{
    // read samples from buffer, run through frame synchronizer
    unsigned int i;
    for (i=0; i<10; i++) {
        // grab data from port
        gport_consume(_q->port_rx,(void*)_q->data_rx,512);

        // run through frame synchronizer
        flexframesync_execute(_q->fs, _q->data_rx, 512);
    }

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

    // re-allocate memory arrays if necessary
    if (q->rx_data_len != payload_len) {
        q->rx_data_len = payload_len;
        q->rx_data = (unsigned char*) realloc(q->rx_data, q->rx_data_len*sizeof(unsigned char));
    }

    // copy data to internal memory array
    memmove(q->rx_header, _rx_header,  14);
    memmove(q->rx_data,   _rx_payload, q->rx_data_len);
    
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

    // create frame generator
    _q->fgprops.rampup_len = 64;
    _q->fgprops.phasing_len = 64;
    _q->fgprops.payload_len = _q->packet_len;    // NOTE : payload_len for frame is packet_len
    _q->fgprops.mod_scheme = MOD_QPSK;
    _q->fgprops.mod_bps = 2;
    _q->fgprops.rampdn_len = 64;
    _q->fg = flexframegen_create(&_q->fgprops);
    flexframegen_print(_q->fg);

    // create frame synchronizer
    framesyncprops_init_default(&_q->fsprops);
    _q->fsprops.squelch_threshold = -30.0f;
    _q->fs = flexframesync_create(&_q->fsprops, ping_callback, (void*)_q);

    // create interpolator
    unsigned int m=3;
    float beta=0.7f;
    _q->interp = interp_crcf_create_rrc(2,m,beta,0);

    // allocate memory
    _q->rx_data_len = 1024;
    _q->rx_data = (unsigned char*) malloc(_q->rx_data_len*sizeof(unsigned char));
}

void pingdata_destroy(pingdata * _q)
{
    // destroy packetizers
    packetizer_destroy(_q->p_enc);
    packetizer_destroy(_q->p_dec);

    // destroy frame generator
    flexframegen_destroy(_q->fg);

    // destroy frame synchronizer
    flexframesync_destroy(_q->fs);

    // destroy interpolator
    interp_crcf_destroy(_q->interp);

    // free memory
    free(_q->rx_data);
}

