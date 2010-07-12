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

#define IQPR_PACKET_TYPE_DATA   (0)
#define IQPR_PACKET_TYPE_ACK    (1)
#define IQPR_PACKET_TYPE_NACK   (2)

#define IQPR_RX_NULL            (0) // not waiting for anything
#define IQPR_RX_WAIT_FOR_DATA   (1) // receiver waiting for data packet
#define IQPR_RX_WAIT_FOR_ACK    (2) // receiver waiting for ack

typedef struct iqpr_s * iqpr;

iqpr iqpr_create(unsigned int _node_id);
void iqpr_destroy(iqpr _q);
void iqpr_txpacket(iqpr _q,
                   unsigned int _pid,
                   unsigned char * _payload,
                   unsigned int _payload_len);
void iqpr_txack(iqpr _q, unsigned int _pid);
void iqpr_rxpacket(iqpr _q);

// wait for data packet, returning -1 if not found pid otherwise
int iqpr_wait_for_data(iqpr _q,
                       unsigned char * _payload,
                       unsigned int * _payload_len);
int iqpr_wait_for_ack(iqpr _q, unsigned int _pid);
int iqpr_mac_clear(iqpr _q);

// ports
void iqpr_connect_txport(iqpr _q, gport _p);
void iqpr_connect_rxport(iqpr _q, gport _p);

// internal methods
int iqpr_callback(unsigned char * _rx_header,
                         int _rx_header_valid,
                         unsigned char * _rx_payload,
                         unsigned int _rx_payload_len,
                         framesyncstats_s _stats,
                         void * _userdata);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif // __IQPR_H__

