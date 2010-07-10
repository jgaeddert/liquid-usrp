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

#define NODE_MASTER         (0)
#define NODE_SLAVE          (1)

#define PACKET_TYPE_DATA    (0)
#define PACKET_TYPE_ACK     (1)
#define PACKET_TYPE_NACK    (2)

void usage() {
    printf("ping usage:\n");
    printf("  u,h   :   usage/help\n");
    printf("  f     :   frequency [Hz], default: 462 MHz\n");
    printf("  b     :   bandwidth [Hz], default: 100 kHz\n");
    printf("  m/s   :   designate node as master/slave, default: slave\n");
}

// ping data structure
struct ping_s {
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
    unsigned char * rx_data;        // received payload data
    unsigned int rx_data_len;       // received payload data length
    unsigned char rx_header[14];    // received header
    unsigned int rx_packet_id;      // received packet identifier
    framesyncprops_s fsprops;       // frame synchronizer properties
    flexframesync fs;               // frame synchronizer
    std::complex<float> rx_buffer[512];   // rx data buffer
    unsigned int rx_ack_pid;        // receiver ack pid
    int rx_ack_found;               // receiver ack found flag

    // transmitter
    packetizer p_enc;               // packet encoder
    unsigned char * tx_data;        // transmitted payload data
    unsigned int tx_data_len;       // transmitted payload data length
    unsigned char tx_header[14];    // transmitted header
    unsigned int tx_packet_id;      // transmitted packet identifier
    flexframegenprops_s fgprops;    // frame generator properties
    flexframegen fg;                // frame generator
    interp_crcf interp;             // matched filter

    // packet manager
    unsigned int num_timeouts;      // running timeout counter
};

typedef struct ping_s * ping;

static int ping_callback(unsigned char * _rx_header,
                         int _rx_header_valid,
                         unsigned char * _rx_payload,
                         unsigned int _rx_payload_len,
                         framesyncstats_s _stats,
                         void * _userdata);
ping ping_create();
void ping_destroy(ping _q);
void ping_txpacket(ping _q,
                   unsigned int _pid,
                   unsigned char * _payload,
                   unsigned int _payload_len);
void ping_txack(ping _q, unsigned int _pid);
void ping_rxpacket(ping _q);
int ping_wait_for_ack(ping _q, unsigned int _pid);

int main (int argc, char **argv) {
    // options
    float frequency = 462e6f;
    float symbolrate = 100e3f;

    // initialize ping structure
    ping q = ping_create();

    //
    int d;
    while ((d = getopt(argc,argv,"uhf:b:ms")) != EOF) {
        switch (d) {
        case 'u':
        case 'h': usage();                      return 0;
        case 'f': frequency = atof(optarg);     break;
        case 'b': symbolrate = atof(optarg);    break;
        case 'm': q->node_type = NODE_MASTER;   break;
        case 's': q->node_type = NODE_SLAVE;    break;
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
    q->port_tx = usrp->get_tx_port(USRP_CHANNEL);
    q->port_rx = usrp->get_rx_port(USRP_CHANNEL);

    usleep(1000000);

    // start data transfer
    usrp->start_tx(USRP_CHANNEL);
    usrp->start_rx(USRP_CHANNEL);

    unsigned int i;
    unsigned int n;
    if (q->node_type == NODE_MASTER) {
        unsigned int payload_len = 1024;
        unsigned char payload[1024];

        // initialize payload to random data
        for (n=0; n<payload_len; n++)
            payload[n] = rand() % 256;

        for (i=0; i<1000; i++) {

            // transmit packet
            ping_txpacket(q,i,payload,payload_len);

            // wait for acknowledgement
            //ping_wait_for_ack(q,i);
        }
    } else {
        for (i=0; i<1000; i++) {
            //ping_rxpacket(q);
            int ack_found = ping_wait_for_ack(q,300);
            if (ack_found) break;
        }
    }

    // stop data transfer
    usrp->stop_rx(USRP_CHANNEL);
    usrp->stop_tx(USRP_CHANNEL);

    printf("main process complete\n");

    // delete usrp object
    delete usrp;

    // destroy main data object
    ping_destroy(q);

    return 0;
}

void ping_txpacket(ping _q,
                   unsigned int _pid,
                   unsigned char * _payload,
                   unsigned int _payload_len)
{
    unsigned int i;
    float g = 0.1f; // transmit gain

    // configure frame generator
    _q->payload_len = _payload_len;
    _q->packet_len = packetizer_get_packet_length(_q->payload_len, FEC_NONE, FEC_NONE);
    _q->fgprops.payload_len = _q->packet_len;
    flexframegen_setprops(_q->fg, &_q->fgprops);

    // allocate memory for buffers
    //unsigned char payload[_q->payload_len];  // raw data
    unsigned char packet[_q->packet_len];    // encoded message length

    unsigned int frame_len = flexframegen_getframelen(_q->fg);
    std::complex<float> frame[frame_len];
    std::complex<float> mfbuffer[2*frame_len];

    printf("[tx] sending packet %u...\n", _pid);

    // recreate packetizer
    _q->p_enc = packetizer_recreate(_q->p_enc, _q->payload_len, FEC_NONE, FEC_NONE);

    // encode packet
    packetizer_encode(_q->p_enc, _payload, packet);

    // prepare header
    _q->tx_header[0] = (_pid >> 8) & 0xff;
    _q->tx_header[1] = (_pid     ) & 0xff;
    _q->tx_header[2] = (_q->payload_len >> 8) & 0xff;
    _q->tx_header[3] = (_q->payload_len     ) & 0xff;
    _q->tx_header[4] = (unsigned char)(FEC_NONE);
    _q->tx_header[5] = (unsigned char)(FEC_NONE);
    _q->tx_header[6] = PACKET_TYPE_DATA;

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

    //_q->pid++;
}


void ping_txack(ping _q,
                unsigned int _pid)
{
    // configure frame generator
    _q->fgprops.payload_len = 0;
    flexframegen_setprops(_q->fg, &_q->fgprops);

    unsigned int i;
    float g = 0.5f; // transmit gain

    unsigned int frame_len = flexframegen_getframelen(_q->fg);
    std::complex<float> frame[frame_len];
    std::complex<float> mfbuffer[2*frame_len];

    printf("[tx] sending ack for packet %u...\n", _q->pid);

    // prepare header
    _q->tx_header[0] = (_pid >> 8) & 0xff;
    _q->tx_header[1] = (_pid     ) & 0xff;
    _q->tx_header[2] = 0;   // payload length (msb)
    _q->tx_header[3] = 0;   // payload length (lsb)
    _q->tx_header[4] = (unsigned char)(FEC_NONE);
    _q->tx_header[5] = (unsigned char)(FEC_NONE);
    _q->tx_header[6] = PACKET_TYPE_ACK;

    // generate frame
    flexframegen_execute(_q->fg, _q->tx_header, NULL, frame);

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
}


void ping_rxpacket(ping _q)
{
    // read samples from buffer, run through frame synchronizer
    unsigned int i;
    for (i=0; i<10; i++) {
        // grab data from port
        gport_consume(_q->port_rx, (void*)_q->rx_buffer, 512);

        // run through frame synchronizer
        flexframesync_execute(_q->fs, _q->rx_buffer, 512);
    }

}

int ping_wait_for_ack(ping _q,
                      unsigned int _pid)
{
    // set ack to trigger on a specific packet id
    _q->rx_ack_pid = _pid;
    _q->rx_ack_found = 0;

    // read samples from buffer, run through frame synchronizer
    unsigned int i;
    for (i=0; i<10; i++) {
        // grab data from port
        gport_consume(_q->port_rx, (void*)_q->rx_buffer, 512);

        // run through frame synchronizer
        flexframesync_execute(_q->fs, _q->rx_buffer, 512);

        // check status flag
        if (_q->rx_ack_found) {
            printf(" received ack for packet %u!\n", _pid);
            return 1;
        }
    }

    // ack was never received
    return 0;
}

// 
// ping internal methods
//

static int ping_callback(unsigned char * _rx_header,
                         int _rx_header_valid,
                         unsigned char * _rx_payload,
                         unsigned int _rx_payload_len,
                         framesyncstats_s _stats,
                         void * _userdata)
{
    ping q = (ping) _userdata;

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
    if (verbose) printf("packet id: %6u", packet_id);
    unsigned int payload_len = (_rx_header[2] << 8 | _rx_header[3]);
    fec_scheme fec0 = (fec_scheme)(_rx_header[4]);
    fec_scheme fec1 = (fec_scheme)(_rx_header[5]);

    unsigned int packet_type = _rx_header[6];
    if (verbose) {
        switch (packet_type) {
        case PACKET_TYPE_DATA:  printf("(data)\n"); break;
        case PACKET_TYPE_ACK:   printf("(ack)\n");  break;
        case PACKET_TYPE_NACK:  printf("(nack)\n"); break;
        default:
            printf("\n");
            fprintf(stderr,"error: ping_callback(), invaid packet type: %u\n", packet_type);
        }
    }

    if (packet_type != PACKET_TYPE_DATA) {
        // TODO : check to see if we were waiting for an ack
        
        // check to see if this packet matches the one we are waiting for
        // TODO : check to see if source/destination ids match as well
        if (packet_id == q->rx_ack_pid) {
            // set status flag
            q->rx_ack_found = 1;
        }

        return 0;
    }

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


ping ping_create()
{
    ping q = (ping) malloc(sizeof(struct ping_s));

    q->continue_running = 1;
    q->node_type = NODE_MASTER;
    q->payload_len = 1024;
    q->packet_len = packetizer_get_packet_length(q->payload_len, FEC_NONE, FEC_NONE);
    q->pid = 0;

    // create packetizers
    q->p_enc = packetizer_create(q->payload_len, FEC_NONE, FEC_NONE);
    q->p_dec = packetizer_create(q->payload_len, FEC_NONE, FEC_NONE);

    // create frame generator
    q->fgprops.rampup_len = 64;
    q->fgprops.phasing_len = 64;
    q->fgprops.payload_len = q->packet_len;    // NOTE : payload_len for frame is packet_len
    q->fgprops.mod_scheme = MOD_QPSK;
    q->fgprops.mod_bps = 2;
    q->fgprops.rampdn_len = 64;
    q->fg = flexframegen_create(&q->fgprops);
    flexframegen_print(q->fg);

    // create frame synchronizer
    framesyncprops_init_default(&q->fsprops);
    q->fsprops.squelch_threshold = -30.0f;
    q->fs = flexframesync_create(&q->fsprops, ping_callback, (void*)q);

    // create interpolator
    unsigned int m=3;
    float beta=0.7f;
    q->interp = interp_crcf_create_rrc(2,m,beta,0);

    // allocate memory
    q->rx_data_len = 1024;
    q->rx_data = (unsigned char*) malloc(q->rx_data_len*sizeof(unsigned char));

    return q;
}

void ping_destroy(ping _q)
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

