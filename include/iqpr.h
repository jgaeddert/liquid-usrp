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

#ifndef __IQPR_H__
#define __IQPR_H__

#ifdef __cplusplus
extern "C" {
#   define LIQUID_USE_COMPLEX_H 0
#else
#   define LIQUID_USE_COMPLEX_H 1
#endif /* __cplusplus */

#include <liquid/liquid.h>

#define IQPR_PACKET_TYPE_DATA       (0)
#define IQPR_PACKET_TYPE_ACK        (1)
#define IQPR_PACKET_TYPE_CONTROL    (2)

#define IQPR_RX_NULL            (0) // not waiting for anything
#define IQPR_RX_WAIT_FOR_DATA   (1) // receiver waiting for data packet
#define IQPR_RX_WAIT_FOR_ACK    (2) // receiver waiting for ack


// iqpr packet header descriptor
struct iqprheader_s {
    unsigned int pid;           // [0,1] packet identifier
    unsigned int packet_type;   // [2]   packet type (data, ack, etc.)

    unsigned int node_src;      // [3]   source node id
    unsigned int node_dst;      // [4]   destination node id

    unsigned char userdata[3];  // [5,6,7]  remaining user data
};

// encode header (structure > array)
//  _q      :   iqpr heade structure
//  _header :   8-byte header array
void iqprheader_encode(iqprheader_s * _q, unsigned char * _header);

// decode header (array > structure))
//  _q      :   iqpr heade structure
//  _header :   8-byte header array
void iqprheader_decode(iqprheader_s * _q, unsigned char * _header);


// 
// iqpr object interface declarations
//

typedef struct iqpr_s * iqpr;

// create iqpr object
//  _node_id    :   id of this node
//  _tx_port    :   transmitter port
//  _rx_port    :   receiver port
iqpr iqpr_create(unsigned int _node_id,
                 gport _port_tx,
                 gport _port_rx);

// destroy iqpr object
void iqpr_destroy(iqpr _q);

// print iqpr object internals
void iqpr_print(iqpr _q);

// set verbosity on/off
void iqpr_setverbose(iqpr _q, int _verbose);

// set transmit gain (linear)
void iqpr_settxgain(iqpr _q, float _txgain);

// transmit packet
//  _q              :   iqpr object
//  _payload        :   payload data
//  _payload_len    :   number of bytes in payload
//  _ms             :   modulation scheme
//  _bps            :   modulation depth
//  _fec0           :   inner fec scheme
//  _fec1           :   outer fec scheme
void iqpr_txpacket(iqpr _q,
                   iqprheader_s * _tx_header,
                   unsigned char * _payload,
                   unsigned int _payload_len,
                   modulation_scheme _ms,
                   unsigned int _bps,
                   fec_scheme _fec0,
                   fec_scheme _fec1);

// transmit ACK packet (acknowledgement) on packet [pid]
void iqpr_txack(iqpr _q, unsigned int _pid);

// wait for data packet, returning 1 if found, 0 if not
//  _q              :   iqpr object
//  _payload        :   payload data
//  _payload_len    :   number of bytes in payload
//  _header         :   received header structure
//  _stats          :   received frame statistics
int iqpr_wait_for_packet(iqpr _q,
                         unsigned char ** _payload,
                         unsigned int * _payload_len,
                         iqprheader_s * _header,
                         framesyncstats_s * _stats);

// wait for ACK packet (acknowlegement)
//  _q              :   iqpr object
//  _pid            :   packet identifier to wait for
//  _header         :   received header structure
//  _stats          :   received frame statistics
int iqpr_wait_for_ack(iqpr _q,
                      unsigned int _pid,
                      iqprheader_s * _header,
                      framesyncstats_s * _stats);

// get channel rssi, estimated on _num_samples samples
float iqpr_mac_getrssi(iqpr _q,
                       unsigned int _num_samples);


// 
// internal methods
//

// iqpr internal callback method
int iqpr_callback(unsigned char * _rx_header,
                  int _rx_header_valid,
                  unsigned char * _rx_payload,
                  unsigned int _rx_payload_len,
                  int _rx_payload_valid,
                  framesyncstats_s _stats,
                  void * _userdata);


#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif // __IQPR_H__

