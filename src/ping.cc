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

#include "iqpr.h"

#define PING_NODE_MASTER    (0)
#define PING_NODE_SLAVE     (1)

#define PING_PACKET_DATA    (59)
#define PING_PACKET_ACK     (77)

void usage() {
    printf("ping usage:\n");
    printf("  u,h   :   usage/help\n");
    printf("  f     :   frequency [Hz], default: 462 MHz\n");
    printf("  b     :   bandwidth [Hz], default: 100 kHz\n");
    printf("  M/S   :   designate node as master/slave, default: slave\n");
    printf("  N     :   number of packets, default: 1000\n");
    printf("  A     :   [master] max. number of tx attempts, default: 100\n");
    printf("  n     :   [master] payload length (bytes), default: 200\n");
    printf("  m     :   [master] mod. scheme: <psk>, dpsk, ask, qam, apsk...\n");
    printf("  p     :   [master] mod. depth: <1>,2,...8\n");
    printf("  c     :   [master] fec coding scheme (inner)\n");
    printf("  k     :   [master] fec coding scheme (outer)\n");
    printf("  v/q   :   set verbose/quiet mode, default: verbose\n");
}

int main (int argc, char **argv) {
    // options
    float frequency = 462e6f;
    float symbolrate = 160e3f;
    unsigned int num_packets = 1000;
    unsigned int node_type = PING_NODE_SLAVE;
    int verbose = 1;

    // master node options
    unsigned int payload_len=200;               // payload length (bytes)
    unsigned int max_num_attempts = 100;        // maximum number of tx attempts
    crc_scheme check    = LIQUID_CRC_16;        // data validity check
    fec_scheme fec0     = LIQUID_FEC_NONE;      // inner FEC scheme
    fec_scheme fec1     = LIQUID_FEC_HAMMING74; // outer FEC scheme
    modulation_scheme mod_scheme = LIQUID_MODEM_QAM;    // modulation scheme
    unsigned int mod_depth = 2;                         // modulation depth

    //
    int d;
    while ((d = getopt(argc,argv,"uhf:b:N:A:MSn:m:p:c:k:vq")) != EOF) {
        switch (d) {
        case 'u':
        case 'h': usage();                          return 0;
        case 'f': frequency = atof(optarg);         break;
        case 'b': symbolrate = atof(optarg);        break;
        case 'N': num_packets = atoi(optarg);       break;
        case 'A': max_num_attempts = atoi(optarg);  break;
        case 'M': node_type = PING_NODE_MASTER;     break;
        case 'S': node_type = PING_NODE_SLAVE;      break;
        case 'n': payload_len = atoi(optarg);       break;
        case 'm': mod_scheme = liquid_getopt_str2mod(optarg);   break;
        case 'p': mod_depth = atoi(optarg);                     break;
        case 'c': fec0 = liquid_getopt_str2fec(optarg);         break;
        case 'k': fec1 = liquid_getopt_str2fec(optarg);         break;
        case 'v': verbose = 1;                                  break;
        case 'q': verbose = 0;                                  break;
        default:
            fprintf(stderr,"error: %s, unsupported option\n", argv[0]);
            exit(1);
        }
    }

    // initialize iqpr structure
    iqpr q = iqpr_create();

    // set rx parameters
    iqpr_set_rx_gain(q, 20);
    iqpr_set_rx_rate(q, symbolrate);
    iqpr_set_rx_freq(q, frequency);

    // set tx parameters
    iqpr_set_tx_gain(q, -20);
    iqpr_set_tx_rate(q, symbolrate);
    iqpr_set_tx_freq(q, frequency);

    // other options
    iqpr_unset_verbose(q);

    // sleep for a small time before starting tx/rx processes
    usleep(1000000);

    // 
    // receiver properties
    //
    unsigned int    timespec = 500;
    unsigned char * rx_header = NULL;
    int             rx_header_valid;
    unsigned char * rx_payload = NULL;
    unsigned int    rx_payload_len;
    int             rx_payload_valid;
    framesyncstats_s stats;
    unsigned int rx_pid = 0;

    //
    // transmitter properties
    //
    flexframegenprops_s fgprops;
    flexframegenprops_init_default(&fgprops);
    fgprops.rampup_len   = 40;
    fgprops.phasing_len  = 80;
    fgprops.check        = check;
    fgprops.fec0         = fec0;
    fgprops.fec1         = fec1;
    fgprops.mod_scheme   = mod_scheme;
    fgprops.mod_bps      = mod_depth;
    fgprops.rampdn_len   = 40;
    unsigned int tx_pid;
    unsigned char tx_header[14];
    unsigned int  tx_payload_len = 200;
    unsigned char tx_payload[tx_payload_len];

    unsigned int n;
    unsigned int num_attempts = 0;

    printf("ping: starting node as %s\n", node_type == PING_NODE_MASTER ? "master" : "slave");

    iqpr_rx_start(q);
    if (node_type == PING_NODE_MASTER) {
        // 
        // MASTER NODE
        //
        int ack_received;

        for (tx_pid=0; tx_pid<num_packets; tx_pid++) {

            // initialize header
            tx_header[0] = (tx_pid >> 8) & 0xff;
            tx_header[1] = (tx_pid     ) & 0xff;
            tx_header[2] = PING_PACKET_DATA;
            for (n=3; n<14; n++)
                tx_header[n] = rand() & 0xff;

            // initialize payload to random data
            for (n=0; n<tx_payload_len; n++)
                tx_payload[n] = rand() % 256;

            ack_received = 0;
            num_attempts = 0;
            do {
                num_attempts++;

                // transmit packet
                printf("transmitting packet %6u/%6u (attempt %4u/%4u) %c\n",
                        tx_pid, num_packets, num_attempts, max_num_attempts,
                        num_attempts > 1 ? '*' : ' ');
                //iqpr_txpacket(q,&tx_header,payload,payload_len,ms,bps,fec0,fec1);
                iqpr_txpacket(q, tx_header, tx_payload, tx_payload_len, &fgprops);

                //usleep(4000);

                // wait for acknowledgement
                unsigned int timer=0;
                ack_received=0;
                while (!ack_received && timer < 25000) {
                    int packet_received =
                    iqpr_rxpacket(q, timespec,
                                  &rx_header,
                                  &rx_header_valid,
                                  &rx_payload,
                                  &rx_payload_len,
                                  &rx_payload_valid,
                                  &stats);
                    timer += timespec;

                    if (packet_received) {
                        rx_pid = (rx_header[0] << 8) | rx_header[1];

                        if (!rx_header_valid) {
                            printf("  rx header invalid!\n");
                        } else if (rx_header[2] != PING_PACKET_ACK) {
                            // effectively ignore our own transmitted signal
                            //printf("  wrong packet type (got %u, expected %u)\n", rx_header[2], PING_PACKET_ACK);
                        } else if (!rx_payload_valid) {
                            printf("  rx payload invalid!\n");
                        } else if (rx_pid != tx_pid) {
                            printf("  ack pid (%4u) does not match tx pid\n", rx_pid);
                        } else {
                            ack_received = 1;
                        }
                    }
                }

                //ack_received = packet_received && rx_pid == tx_pid && rx_header[2] == PING_PACKET_ACK;

                if (ack_received) {
                    //printf("ACK RECEIVED [%4u]!\n", rx_pid);
                    //usleep(10);
                    break;
                } else {
                    //printf("TIMEOUT\n");
                    //goto end;
                }
            } while (!ack_received && (num_attempts < max_num_attempts) );

            if (num_attempts == max_num_attempts) {
                printf("transmitter reached maximum number of attemts; bailing\n");
                break;
            }
        }
    } else {
        // 
        // SLAVE NODE
        //
        fgprops.check        = LIQUID_CRC_NONE;
        fgprops.mod_scheme   = LIQUID_MODEM_QPSK;
        fgprops.mod_bps      = 2;

        int packet_found = 0;
        do {
            // wait for data packet
            do {
                // attempt to receive data packet
                packet_found =
                iqpr_rxpacket(q,
                              timespec,
                              &rx_header,
                              &rx_header_valid,
                              &rx_payload,
                              &rx_payload_len,
                              &rx_payload_valid,
                              &stats);
            } while (!packet_found);
            
            if (!rx_header_valid) {
                printf("  header crc : FAIL\n");
                continue;
            } else if (rx_header[2] != PING_PACKET_DATA) {
                // effectively ignore our own transmitted signal
                //printf("  invalid packet type\n");
                continue;
            }
            
            rx_pid = (rx_header[0] << 8) | rx_header[1];
            //if (rx_pid == 1) break;

            if (!rx_payload_valid) {
                printf("  payload crc : FAIL [%4u]\n", rx_pid);
                continue;
            }

            printf("  ping received %4u data bytes on packet [%4u] rssi : %12.4f dB\n",
                    rx_payload_len,
                    rx_pid,
                    stats.rssi);

            // print received frame statistics
            //if (verbose) framesyncstats_print(&stats);

            // transmit acknowledgement
            tx_header[0] = rx_header[0];
            tx_header[1] = rx_header[1];
            tx_header[2] = PING_PACKET_ACK;  // ACK code
            for (n=3; n<14; n++)
                tx_header[n] = rand() & 0xff;

            unsigned char ack_payload[10];
            for (n=0; n<10; n++)
                ack_payload[n] = rand() & 0xff;
            iqpr_txpacket(q, tx_header, ack_payload, 10, &fgprops);

        } while (rx_pid != num_packets-1);
    }
    iqpr_rx_stop(q);

    // TODO : stop timer

    // sleep for a small time before stopping tx/rx processes
    usleep(100000);

    printf("main process complete\n");

    // destroy main data object
    iqpr_destroy(q);

    return 0;
}

