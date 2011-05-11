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

#define NODE_MASTER         (0)
#define NODE_SLAVE          (1)

void usage() {
    printf("ping usage:\n");
    printf("  u,h   :   usage/help\n");
    printf("  f     :   frequency [Hz], default: 462 MHz\n");
    printf("  b     :   bandwidth [Hz], default: 100 kHz\n");
    printf("  n     :   number of packets, default: 1000\n");
    printf("  a     :   number of tx attempts (master), default: 100\n");
    printf("  m/s   :   designate node as master/slave, default: slave\n");
    printf("  v/q   :   set verbose/quiet mode, default: verbose\n");
}

int main (int argc, char **argv) {
    // options
    float frequency = 462e6f;
    float symbolrate = 160e3f;
    unsigned int num_packets = 1000;
    unsigned int max_num_attempts = 100;    // maximum number of tx attempts
    unsigned int node_type = NODE_MASTER;
    int verbose = 1;

    //
    int d;
    while ((d = getopt(argc,argv,"uhf:b:n:a:msvq")) != EOF) {
        switch (d) {
        case 'u':
        case 'h': usage();                          return 0;
        case 'f': frequency = atof(optarg);         break;
        case 'b': symbolrate = atof(optarg);        break;
        case 'n': num_packets = atoi(optarg);       break;
        case 'a': max_num_attempts = atoi(optarg);  break;
        case 'm': node_type = NODE_MASTER;          break;
        case 's': node_type = NODE_SLAVE;           break;
        case 'v': verbose = 1;                      break;
        case 'q': verbose = 0;                      break;
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
    unsigned int    timespec = 10000;
    unsigned char * rx_header = NULL;
    int             rx_header_valid;
    unsigned char * rx_payload = NULL;
    unsigned int    rx_payload_len;
    int             rx_payload_valid;
    framesyncstats_s stats;
    unsigned char rx_pid = 0;

    //
    // transmitter properties
    //
    flexframegenprops_s fgprops;
    flexframegenprops_init_default(&fgprops);
    fgprops.rampup_len   = 40;
    fgprops.phasing_len  = 40;
    fgprops.check        = LIQUID_CRC_32;
    fgprops.fec0         = LIQUID_FEC_NONE;
    fgprops.fec1         = LIQUID_FEC_NONE;
    fgprops.mod_scheme   = LIQUID_MODEM_QAM;
    fgprops.mod_bps      = 4;
    fgprops.rampdn_len   = 40;
    unsigned char tx_pid;
    unsigned char tx_header[14];
    unsigned int  tx_payload_len = 1024;
    unsigned char tx_payload[tx_payload_len];

    unsigned int n;
    unsigned int num_attempts = 0;

    if (node_type == NODE_MASTER) {
        // 
        // MASTER NODE
        //
        int ack_received;

        for (tx_pid=0; tx_pid<num_packets; tx_pid++) {

            // initialize header
            tx_header[0] = (tx_pid >> 8) & 0xff;
            tx_header[1] = (tx_pid     ) & 0xff;
            tx_header[2] = 59;
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

                // wait for acknowledgement
                int packet_received =
                iqpr_rxpacket(q, timespec,
                              &rx_header,
                              &rx_header_valid,
                              &rx_payload,
                              &rx_payload_len,
                              &rx_payload_valid,
                              &stats);

                rx_pid = (rx_header[0] << 8) | rx_header[1];
                ack_received = packet_received && rx_pid == tx_pid && rx_header[2] == 77;

                if (ack_received)
                    break;
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
            
            if (!rx_header_valid || !rx_payload_valid || rx_header[2] != 59) {
                printf("invalid packet\n");
                continue;
            }

            rx_pid = (rx_header[0] << 8) | rx_header[1];

            printf("  ping received %4u data bytes on packet [%4u]\n",
                    rx_payload_len,
                    rx_pid);

            // print received frame statistics
            if (verbose) framesyncstats_print(&stats);

            // transmit acknowledgement
            tx_header[0] = rx_header[0];
            tx_header[1] = rx_header[1];
            tx_header[2] = 77;  // ACK code
            iqpr_txpacket(q, tx_header, NULL, 0, &fgprops);

        } while (rx_pid != num_packets-1);
    }

    // TODO : stop timer

    // sleep for a small time before stopping tx/rx processes
    usleep(100000);

    printf("main process complete\n");

    // destroy main data object
    iqpr_destroy(q);

    return 0;
}

