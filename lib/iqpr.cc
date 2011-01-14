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
// iqpr: l(iq)uid (p)acket (r)adio
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
    unsigned int node_id;           // node identifier

    // receiver
    unsigned char * rx_data;        // received payload data
    unsigned int rx_data_len;       // received payload data length
    iqprheader_s rx_header;         // received header
    framesyncprops_s fsprops;       // frame synchronizer properties
    flexframesync fs;               // frame synchronizer
    std::complex<float> rx_buffer[512];   // rx data buffer
    unsigned int rx_packet_pid;     // receiver packet pid
    int rx_packet_found;            // receiver packet found flag
    unsigned int rx_state;          // receiver state (waiting on a packet or ack?)
    framesyncstats_s rx_stats;      // frame synchronizer stats

    // transmitter
    unsigned char * tx_data;        // transmitted payload data
    unsigned int tx_data_len;       // transmitted payload data length
    iqprheader_s tx_header;         // transmitted header
    flexframegenprops_s fgprops;    // frame generator properties
    flexframegen fg;                // frame generator
    interp_crcf interp;             // matched filter
    float tx_gain;                  // transmit gain

    // debugging
    int verbose;
};


// create iqpr object
iqpr iqpr_create(unsigned int _node_id,
                 gport _port_tx,
                 gport _port_rx)
{
    // allocate memory for main object
    iqpr q = (iqpr) malloc(sizeof(struct iqpr_s));
    q->node_id = _node_id;  // node id
    q->port_tx = _port_tx;  // transmit port
    q->port_rx = _port_rx;  // receive port

    // create frame generator
    flexframegenprops_init_default(&q->fgprops);
    q->fgprops.rampup_len   = 16;
    q->fgprops.phasing_len  = 64;
    q->fgprops.payload_len  = 0;
    q->fgprops.check        = CRC_NONE;
    q->fgprops.fec0         = FEC_NONE;
    q->fgprops.fec1         = FEC_NONE;
    q->fgprops.mod_scheme   = MOD_QPSK;
    q->fgprops.mod_bps      = 2;
    q->fgprops.rampdn_len   = 16;
    q->fg = flexframegen_create(&q->fgprops);
    flexframegen_print(q->fg);

    // create frame synchronizer
    framesyncprops_init_default(&q->fsprops);
    q->fsprops.squelch_threshold = -37.0f;
    q->fsprops.squelch_enabled = 1;
    q->fs = flexframesync_create(&q->fsprops, iqpr_callback, (void*)q);

    // create interpolator
    unsigned int m=3;
    float beta=0.7f;
    q->interp = interp_crcf_create_rrc(2,m,beta,0);

    // set transmit gain
    q->tx_gain = 1.0f;

    // allocate memory for received data
    q->rx_data_len = 1024;
    q->rx_data = (unsigned char*) malloc(q->rx_data_len*sizeof(unsigned char));

    // debugging
    q->verbose = 0;

    return q;
}

void iqpr_destroy(iqpr _q)
{
    // destroy frame generator
    flexframegen_destroy(_q->fg);

    // destroy frame synchronizer
    flexframesync_destroy(_q->fs);

    // destroy interpolator
    interp_crcf_destroy(_q->interp);

    // free memory
    free(_q->rx_data);
}

void iqpr_print(iqpr _q)
{
    printf("iqpr:\n");
}

void iqpr_setverbose(iqpr _q, int _verbose)
{
    _q->verbose = _verbose ? 1 : 0;
}

void iqpr_settxgain(iqpr _q, float _txgain)
{
    // validate input
    if (_txgain <= 0.0f || _txgain > 1.0f) {
        fprintf(stderr,"error: iqpr_settxgain(), tx gain must be in (0,1]\n");
        exit(1);
    }
    _q->tx_gain = _txgain;
}

void iqpr_txpacket(iqpr _q,
                   unsigned int _pid,
                   unsigned char * _payload,
                   unsigned int _payload_len,
                   modulation_scheme _ms,
                   unsigned int _bps,
                   fec_scheme _fec0,
                   fec_scheme _fec1)
{
    unsigned int i;

    // prepare header
    _q->tx_header.pid = _pid;
    _q->tx_header.packet_type = IQPR_PACKET_TYPE_DATA;
    _q->tx_header.node_src = _q->node_id;
    _q->tx_header.node_dst = 0;

    // encode header
    unsigned char header[8];
    iqprheader_encode(&_q->tx_header, header);

    // configure frame generator
    _q->fgprops.payload_len = _payload_len;
    _q->fgprops.mod_scheme  = _ms;
    _q->fgprops.mod_bps     = _bps;
    _q->fgprops.check       = CRC_32;
    _q->fgprops.fec0        = _fec0;
    _q->fgprops.fec1        = _fec1;
    flexframegen_setprops(_q->fg, &_q->fgprops);

    unsigned int frame_len = flexframegen_getframelen(_q->fg);
    unsigned int buffer_len = 64;
    std::complex<float> frame[frame_len];       // frame
    std::complex<float> mfbuffer[2*buffer_len]; // frame (interpolated)

    //printf("[tx] sending packet %u...\n", _pid);

    // generate frame
    flexframegen_execute(_q->fg, header, _payload, frame);

    // run interpolator in blocks
    unsigned int n=frame_len;   // total number of frame samples remaining
    unsigned int k=0;     // block counter
    while (n > 0) {
        // size of this block (default 64, otherwise remaining symbols)
        unsigned int t = (n < buffer_len) ? n : buffer_len;
        for (i=0; i<t; i++)
            interp_crcf_execute(_q->interp, frame[buffer_len*k + i]*_q->tx_gain, &mfbuffer[2*i]);
        
        // update counters
        n -= t;     // reduce number of samples remaining
        k++;        // increment block counter

        // push samples to port
        gport_produce(_q->port_tx, (void*)mfbuffer, 2*t);
    }

    // flush interpolator with zeros, min(64,buffer_len)
    n = buffer_len < 64 ? buffer_len : 64;
    for (i=0; i<n; i++)
        interp_crcf_execute(_q->interp, 0, &mfbuffer[2*i]);

    gport_produce(_q->port_tx, (void*)mfbuffer, 2*n);
}


void iqpr_txack(iqpr _q,
                unsigned int _pid)
{
    // configure frame generator
    _q->fgprops.payload_len = 0;
    flexframegen_setprops(_q->fg, &_q->fgprops);

    unsigned int i;

    unsigned int frame_len = flexframegen_getframelen(_q->fg);
    std::complex<float> frame[frame_len];
    std::complex<float> mfbuffer[2*frame_len];

    if (_q->verbose) printf("[tx] sending ack for packet %u...\n", _pid);

    // prepare header
    _q->tx_header.pid = _pid;
    _q->tx_header.packet_type = IQPR_PACKET_TYPE_ACK;
    _q->tx_header.node_src = _q->node_id;
    _q->tx_header.node_dst = 0;

    unsigned char header[8];
    iqprheader_encode(&_q->tx_header, header);

    // configure frame generator
    _q->fgprops.payload_len = 0;
    _q->fgprops.mod_scheme  = MOD_BPSK;
    _q->fgprops.mod_bps     = 1;
    _q->fgprops.check       = CRC_NONE;
    _q->fgprops.fec0        = FEC_NONE;
    _q->fgprops.fec1        = FEC_NONE;
    flexframegen_setprops(_q->fg, &_q->fgprops);

    // generate frame
    flexframegen_execute(_q->fg, header, NULL, frame);

    // run interpolator
    for (i=0; i<frame_len; i++) {
        interp_crcf_execute(_q->interp, frame[i]*_q->tx_gain, &mfbuffer[2*i]);
    }

    // produce data in buffer
    gport_produce(_q->port_tx, (void*)mfbuffer, 2*frame_len);

    // flush interpolator with zeros
    for (i=0; i<64; i++) {
        interp_crcf_execute(_q->interp, 0, &mfbuffer[2*i]);
    }

    gport_produce(_q->port_tx, (void*)mfbuffer, 128);
}

#if 0
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
#endif

int iqpr_wait_for_packet(iqpr _q,
                         unsigned char ** _payload,
                         unsigned int * _payload_len,
                         iqprheader_s * _header,
                         framesyncstats_s * _stats)
{
    _q->rx_state = IQPR_RX_WAIT_FOR_DATA;
    _q->rx_packet_found = 0;

    // read samples from buffer, run through frame synchronizer
    unsigned int i;
    for (i=0; i<10; i++) {
        // grab data from port
        gport_consume(_q->port_rx, (void*)_q->rx_buffer, 512);

        // run through frame synchronizer
        flexframesync_execute(_q->fs, _q->rx_buffer, 512);

        // check status flag
        if (_q->rx_packet_found) {
            if (_q->verbose) printf(" received data packet %u!\n", _q->rx_header.pid);

            // set outputs
            *_payload = _q->rx_data;
            *_payload_len = _q->rx_data_len;
            memmove(_header, &_q->rx_header, sizeof(struct iqprheader_s));
            memmove(_stats,  &_q->rx_stats,  sizeof(framesyncstats_s));
            return 1;
        }
    }

    // ack was never received
    return 0;
}


int iqpr_wait_for_ack(iqpr _q,
                      unsigned int _pid,
                      iqprheader_s * _header,
                      framesyncstats_s * _stats)
{
    // set ack to trigger on a specific packet id
    _q->rx_state = IQPR_RX_WAIT_FOR_ACK;
    _q->rx_packet_pid = _pid;
    _q->rx_packet_found = 0;

    // read samples from buffer, run through frame synchronizer
    unsigned int i;
    for (i=0; i<10; i++) {
        // grab data from port
        gport_consume(_q->port_rx, (void*)_q->rx_buffer, 512);

        // run through frame synchronizer
        flexframesync_execute(_q->fs, _q->rx_buffer, 512);

        // check status flag
        if (_q->rx_packet_found) {
            if (_q->verbose) printf(" received ack for packet %u!\n", _pid);

            memmove(_header, &_q->rx_header, sizeof(struct iqprheader_s));
            memmove(_stats,  &_q->rx_stats,  sizeof(framesyncstats_s));
            return 1;
        }
    }

    // ack was never received
    return 0;
}


// determine if MAC is clear
float iqpr_mac_getrssi(iqpr _q,
                       unsigned int _num_samples)
{
    // validate input
    if (_num_samples < 1) {
        fprintf(stderr,"warning: iqpr_mac_getrssi(), must use at least 1 sample\n");
        _num_samples = 1;
    }

    // grab data from port
    gport_consume(_q->port_rx, (void*)_q->rx_buffer, _num_samples);

    // estimate signal level
    unsigned int i;
    float rssi = 0.0;
    for (i=0; i<_num_samples; i++)
        rssi += abs(_q->rx_buffer[i]) * abs(_q->rx_buffer[i]);

    // TODO : fix rssi computation
    rssi = 10*log10f(sqrt(rssi/_num_samples));

    if (_q->verbose) {
        printf("mac_clear(), rssi : %12.8f dB %c\n", rssi,
            (rssi < _q->fsprops.squelch_threshold) ? ' ' : '*');
    }

    return rssi;
}


// 
// iqpr internal methods
//

int iqpr_callback(unsigned char * _rx_header,
                  int _rx_header_valid,
                  unsigned char * _rx_payload,
                  unsigned int _rx_payload_len,
                  int _rx_payload_valid,
                  framesyncstats_s _stats,
                  void * _userdata)
{
    iqpr q = (iqpr) _userdata;

    if (q->verbose) {
        printf("********* callback invoked, ");
        framesyncstats_print(&_stats);
        // ...
    }

    if ( !_rx_header_valid ) {
        if (q->verbose) printf("header crc : FAIL\n");
        return 0;
    }

    // save statistics
    memmove(&q->rx_stats, &_stats, sizeof(framesyncstats_s));

    // decode header
    iqprheader_decode(&q->rx_header, _rx_header);
    if (q->verbose) printf("packet id: %6u", q->rx_header.pid);

    if (q->verbose) {
        switch (q->rx_header.packet_type) {
        case IQPR_PACKET_TYPE_DATA:  printf(" (data)\n"); break;
        case IQPR_PACKET_TYPE_ACK:   printf(" (ack)\n");  break;
        default:
            printf("\n");
            fprintf(stderr,"error: iqpr_callback(), invaid packet type: %u\n", q->rx_header.packet_type);
        }
    }

    if (q->rx_header.packet_type == IQPR_PACKET_TYPE_ACK) {
        // check to see if we were waiting for an ack and
        // check to see if this packet matches the one we are waiting for
        //
        // TODO : check to see if source/destination ids match as well?
        if ( (q->rx_state == IQPR_RX_WAIT_FOR_ACK) && (q->rx_header.pid == q->rx_packet_pid) ) {
            // set status flag
            q->rx_packet_found = 1;
        } else {
            q->rx_packet_found = 0;
        }

        return 0;
    }

    // re-allocate memory arrays if necessary
    if (q->rx_data_len != _rx_payload_len) {
        q->rx_data_len = _rx_payload_len;
        q->rx_data = (unsigned char*) realloc(q->rx_data, q->rx_data_len*sizeof(unsigned char));
    }

    // copy data (regardless of validity)
    memmove(q->rx_data, _rx_payload, _rx_payload_len*sizeof(unsigned char));

    if (!_rx_payload_valid) {
        if (q->verbose) printf("  <<< payload crc fail >>>\n");

        // payload failed check
        return 0;
    }

    // check to see if we were waiting for a data packet
    if (q->rx_state == IQPR_RX_WAIT_FOR_DATA)
        q->rx_packet_found = 1;

    return 0;
}

// encode header
void iqprheader_encode(iqprheader_s * _q,
                       unsigned char * _header)
{
    // prepare header
    _header[0] = (_q->pid >> 8) & 0xff;
    _header[1] = (_q->pid     ) & 0xff;

    _header[6] = _q->packet_type & 0xff;
    _header[7] = (_q->node_src & 0x0f);
    _header[8] = (_q->node_dst & 0x0f) << 4;
}


// decode header
void iqprheader_decode(iqprheader_s * _q,
                       unsigned char * _header)
{
    // decode packet identifier
    _q->pid = (_header[0] << 8) | _header[1];

    // decode packet type
    _q->packet_type = _header[6];

    // decode source and destination nodes
    _q->node_src = (_header[7]     ) & 0x0f;
    _q->node_dst = (_header[7] >> 4) & 0x0f;
}

