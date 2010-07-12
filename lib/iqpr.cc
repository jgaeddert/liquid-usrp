/*
 * Copyright (c) 2010 Joseph Gaeddert
 * Copyright (c) 2010 Virginia Polytechnic Institute & State University
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
// iqpr.cc
//
// iqpr basic data packets back and forth
//

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <complex>
#include <liquid/liquid.h>

#include "iqpr.h"

// iqpr data structure
struct iqpr_s {
    gport port_tx;                  // transmit port
    gport port_rx;                  // receive port

    // common
    unsigned int payload_len;       // payload length (raw data)
    unsigned int packet_len;        // packet length (encoded data)
    unsigned int node_id;           // node identifier

    // receiver
    packetizer p_dec;               // packet decoder
    unsigned char * rx_data;        // received payload data
    unsigned int rx_data_len;       // received payload data length
    iqprheader_s rx_header;         // received header
    framesyncprops_s fsprops;       // frame synchronizer properties
    flexframesync fs;               // frame synchronizer
    std::complex<float> rx_buffer[512];   // rx data buffer
    unsigned int rx_ack_pid;        // receiver ack pid
    int rx_ack_found;               // receiver ack found flag
    unsigned int rx_state;

    // transmitter
    packetizer p_enc;               // packet encoder
    unsigned char * tx_data;        // transmitted payload data
    unsigned int tx_data_len;       // transmitted payload data length
    iqprheader_s tx_header;         // transmitted header
    flexframegenprops_s fgprops;    // frame generator properties
    flexframegen fg;                // frame generator
    interp_crcf interp;             // matched filter
};


iqpr iqpr_create(unsigned int _node_id)
{
    iqpr q = (iqpr) malloc(sizeof(struct iqpr_s));

    q->payload_len = 1024;
    q->packet_len = packetizer_get_packet_length(q->payload_len, FEC_NONE, FEC_NONE);
    q->node_id = _node_id;

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
    q->fs = flexframesync_create(&q->fsprops, iqpr_callback, (void*)q);

    // create interpolator
    unsigned int m=3;
    float beta=0.7f;
    q->interp = interp_crcf_create_rrc(2,m,beta,0);

    // allocate memory
    q->rx_data_len = 1024;
    q->rx_data = (unsigned char*) malloc(q->rx_data_len*sizeof(unsigned char));

    return q;
}

void iqpr_destroy(iqpr _q)
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

void iqpr_txpacket(iqpr _q,
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

    //printf("[tx] sending packet %u...\n", _pid);

    // recreate packetizer
    _q->p_enc = packetizer_recreate(_q->p_enc, _q->payload_len, FEC_NONE, FEC_NONE);

    // encode packet
    packetizer_encode(_q->p_enc, _payload, packet);

    // prepare header
    unsigned char header[14];
    _q->tx_header.pid = _pid;
    _q->tx_header.payload_len = _payload_len;
    _q->tx_header.fec0 = FEC_NONE;
    _q->tx_header.fec1 = FEC_NONE;
    _q->tx_header.packet_type = IQPR_PACKET_TYPE_DATA;
    _q->tx_header.node_src = _q->node_id;
    _q->tx_header.node_dst = 0;
    iqprheader_encode(&_q->tx_header, header);

    // generate frame
    flexframegen_execute(_q->fg, header, packet, frame);

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


void iqpr_txack(iqpr _q,
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

    printf("[tx] sending ack for packet %u...\n", _pid);

    // prepare header
    _q->tx_header.pid = _pid;
    _q->tx_header.payload_len = 0;
    _q->tx_header.fec0 = FEC_NONE;
    _q->tx_header.fec1 = FEC_NONE;
    _q->tx_header.packet_type = IQPR_PACKET_TYPE_ACK;
    _q->tx_header.node_src = _q->node_id;
    _q->tx_header.node_dst = 0;

    unsigned char header[14];
    iqprheader_encode(&_q->tx_header, header);

    // generate frame
    flexframegen_execute(_q->fg, header, NULL, frame);

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


void iqpr_rxpacket(iqpr _q)
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

int iqpr_wait_for_data(iqpr _q,
                       unsigned char ** _payload,
                       unsigned int * _payload_len)
{
    _q->rx_state = IQPR_RX_WAIT_FOR_DATA;
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
            printf(" received data packet %u!\n", _q->rx_header.pid);

            // set output memory pointer
            *_payload = _q->rx_data;
            *_payload_len = _q->rx_data_len;
            return _q->rx_header.pid;
        }
    }

    // ack was never received
    return -1;
}


int iqpr_wait_for_ack(iqpr _q,
                      unsigned int _pid)
{
    // set ack to trigger on a specific packet id
    _q->rx_state = IQPR_RX_WAIT_FOR_ACK;
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
            //printf(" received ack for packet %u!\n", _pid);
            return 1;
        }
    }

    // ack was never received
    return 0;
}


// determine if MAC is clear
int iqpr_mac_clear(iqpr _q)
{
    // grab data from port
    gport_consume(_q->port_rx, (void*)_q->rx_buffer, 512);

    // estimate signal level
    unsigned int i;
    float rssi = 0.0;
    for (i=0; i<512; i++)
        rssi += abs(_q->rx_buffer[i]) * abs(_q->rx_buffer[i]);

    // TODO : fix rssi computation
    rssi = 10*log10f(sqrt(rssi/512));

#if 1
    printf("mac_clear(), rssi : %12.8f dB %c\n", rssi,
        (rssi < _q->fsprops.squelch_threshold) ? ' ' : '*');
#endif

    return (rssi < _q->fsprops.squelch_threshold) ? 1 : 0;
}


// ports


void iqpr_connect_txport(iqpr _q, gport _p)
{
    _q->port_tx = _p;
}

void iqpr_connect_rxport(iqpr _q, gport _p)
{
    _q->port_rx = _p;
}

// 
// iqpr internal methods
//

int iqpr_callback(unsigned char * _rx_header,
                         int _rx_header_valid,
                         unsigned char * _rx_payload,
                         unsigned int _rx_payload_len,
                         framesyncstats_s _stats,
                         void * _userdata)
{
    iqpr q = (iqpr) _userdata;

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

    // decode header
    iqprheader_decode(&q->rx_header, _rx_header);
    if (verbose) printf("packet id: %6u", q->rx_header.pid);

    if (verbose) {
        switch (q->rx_header.packet_type) {
        case IQPR_PACKET_TYPE_DATA:  printf(" (data)\n"); break;
        case IQPR_PACKET_TYPE_ACK:   printf(" (ack)\n");  break;
        case IQPR_PACKET_TYPE_NACK:  printf(" (nack)\n"); break;
        default:
            printf("\n");
            fprintf(stderr,"error: iqpr_callback(), invaid packet type: %u\n", q->rx_header.packet_type);
        }
    }

    if (q->rx_header.packet_type != IQPR_PACKET_TYPE_DATA) {
        // check to see if we were waiting for an ack and
        // check to see if this packet matches the one we are waiting for
        //
        // TODO : check to see if source/destination ids match as well
        if ( (q->rx_state == IQPR_RX_WAIT_FOR_ACK) && (q->rx_header.pid == q->rx_ack_pid) ) {
            // set status flag
            q->rx_ack_found = 1;
        } else {
            q->rx_ack_found = 0;
        }

        return 0;
    }

    // check to see if we were waiting for a data packet
    if (q->rx_state == IQPR_RX_WAIT_FOR_DATA)
        q->rx_ack_found = 1;

    /*
    // TODO: validate fec0,fec1 before indexing fec_scheme_str
    printf("    payload len : %u\n", payload_len);
    printf("    fec0        : %s\n", fec_scheme_str[fec0]);
    printf("    fec1        : %s\n", fec_scheme_str[fec1]);
    */

    // recreate packetizer if necessary
    q->p_dec = packetizer_recreate(q->p_dec,
                                   q->rx_header.payload_len,
                                   q->rx_header.fec0,
                                   q->rx_header.fec1);

    // re-allocate memory arrays if necessary
    if (q->rx_data_len != q->rx_header.payload_len) {
        q->rx_data_len = q->rx_header.payload_len;
        q->rx_data = (unsigned char*) realloc(q->rx_data, q->rx_data_len*sizeof(unsigned char));
    }

    // decode packet
    bool crc_pass = packetizer_decode(q->p_dec, _rx_payload, q->rx_data);
    if (crc_pass) {
    } else {
        if (verbose) printf("  <<< payload crc fail >>>\n");
    }

    return 0;
}

// encode header
void iqprheader_encode(iqprheader_s * _q,
                       unsigned char * _header)
{
    // prepare header
    _header[0] = (_q->pid >> 8) & 0xff;
    _header[1] = (_q->pid     ) & 0xff;
    _header[2] = (_q->payload_len >> 8) & 0xff;
    _header[3] = (_q->payload_len     ) & 0xff;
    _header[4] = (unsigned char)(_q->fec0);
    _header[5] = (unsigned char)(_q->fec1);
    _header[6] = _q->packet_type & 0xff;
    _header[7] = _q->node_src & 0xff;
    _header[8] = _q->node_dst & 0xff;
}


// decode header
void iqprheader_decode(iqprheader_s * _q,
                       unsigned char * _header)
{
    // decode packet identifier
    _q->pid = (_header[0] << 8) | _header[1];

    // decode payload length
    _q->payload_len = (_header[2] << 8) | _header[3];

    // decode fec schemes
    _q->fec0 = (fec_scheme)(_header[4]);
    _q->fec1 = (fec_scheme)(_header[5]);

    // decode packet type
    _q->packet_type = _header[6];

    // decode source and destination nodes
    _q->node_src = _header[7];
    _q->node_dst = _header[8];

}

