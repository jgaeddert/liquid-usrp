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

#include <liquid/liquid.h>
#include <uhd/usrp/single_usrp.hpp>

#define IQPR_PACKET_TYPE_DATA       (0)
#define IQPR_PACKET_TYPE_ACK        (1)
#define IQPR_PACKET_TYPE_CONTROL    (2)

// 
// iqpr object interface declarations
//

typedef struct iqpr_s * iqpr;

// create iqpr object
iqpr iqpr_create();

// destroy iqpr object
void iqpr_destroy(iqpr _q);

// print iqpr object internals
void iqpr_print(iqpr _q);

// set verbosity on/off
void iqpr_set_verbose(iqpr _q);
void iqpr_unset_verbose(iqpr _q);

//
// PHY properties
//

// set transmit/receive hardware gain
void iqpr_set_tx_gain(iqpr _q, float _tx_gain);
void iqpr_set_rx_gain(iqpr _q, float _rx_gain);

#if 0
// set transmit/receive software gain
void iqpr_set_tx_power(iqpr _q, float _tx_power);
void iqpr_set_rx_power(iqpr _q, float _rx_power);
#endif

// set transmit/receive sample rate
void iqpr_set_tx_rate(iqpr _q, float _tx_rate);
void iqpr_set_rx_rate(iqpr _q, float _rx_rate);

// set transmit/receive frequency
void iqpr_set_tx_freq(iqpr _q, float _tx_freq);
void iqpr_set_rx_freq(iqpr _q, float _rx_freq);

// set flexframesync properties (receiver)
void iqpr_rxconfig(iqpr _q, framesyncprops_s * _fsprops);

// start/stop receiver streaming
void iqpr_rx_start(iqpr _q);
void iqpr_rx_stop(iqpr _q);


// 
// low-level functionality
//

// transmit packet (generic)
//  _q              :   iqpr object
//  _header         :   header data [14 bytes]
//  _payload        :   payload data
//  _payload_len    :   number of bytes in payload
//  _fgprops        :   frame generator properties (internal 'payload_len' ignored)
void iqpr_txpacket(iqpr _q,
                   unsigned char * _header,
                   unsigned char * _payload,
                   unsigned int _payload_len,
                   flexframegenprops_s * _fgprops);

// receive data packet with timeout, returning 1 if found, 0 if not
//  _q              :   iqpr object
//  _timespec       :   time specifier
//  _header         :   received header
//  _header_valid   :   header valid?
//  _payload        :   output payload data
//  _payload_len    :   number of bytes in payload
//  _payload_valid  :   payload valid?
//  _stats          :   received frame statistics
int iqpr_rxpacket(iqpr _q,
                  unsigned int _timespec,
                  unsigned char ** _header,
                  int           *  _header_valid,
                  unsigned char ** _payload,
                  unsigned int  *  _payload_len,
                  int           *  _payload_valid,
                  framesyncstats_s * _stats);

//
// higher functionality
//

// packet types:
//  code    name            description
//  0       UDP_PACKET      user datagram packet (no ACK/NACK response)
//  1       TCP_PACKET      transmission control protocol (request ACK/NACK)
//  2       ACK_PACKET      acknowledgement
//  3       NACK_PACKET     negative acknowledgement
//  4       RTS_PACKET      request to send
//  5       CTS_PACKET      clear to send
//  6       CTRL_PACKET     control information

// transmit data packet
void iqpr_txdatat(iqpr _q,
                  unsigned int _pid);
                  // ...

// transmit ACK packet (acknowledgement) on packet [pid]
void iqpr_txack(iqpr _q, unsigned int _pid);

// wait for ACK packet (acknowlegement)
//  _q              :   iqpr object
//  _pid            :   packet identifier to wait for
//  _stats          :   received frame statistics
int iqpr_wait_for_ack(iqpr _q,
                      unsigned int _pid,
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

#if 0
// iqpr packet header descriptor (14 bytes total space)
//  length  name        description
//  2       pid         packet identifier number
//  2       node_src    source node ID
//  2       node_dst    destination node ID
//  1       type        packet type (UDP_PACKET, TCP_PACKET, ACK_PACKET, NACK_PACKET...)
//  xxx     userdata    remaining data reserved for user
struct iqprheader_s {
    unsigned int pid;           // [0,1] packet identifier
    unsigned int packet_type;   // [2]   packet type (data, ack, etc.)

    unsigned int node_src;      // [3]   source node id
    unsigned int node_dst;      // [4]   destination node id

    unsigned char userdata[3];  // [5,6,7]  remaining user data
};

// encode header (structure > array)
//  _q      :   iqpr heade structure
//  _header :   8-byte header array [XXX TODO : make 14-byte array]
void iqprheader_encode(iqprheader_s * _q, unsigned char * _header);

// decode header (array > structure))
//  _q      :   iqpr heade structure
//  _header :   8-byte header array [XXX TODO : make 14-byte array]
void iqprheader_decode(iqprheader_s * _q, unsigned char * _header);

#endif

#endif // __IQPR_H__

