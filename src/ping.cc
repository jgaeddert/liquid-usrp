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
#include "iqpr.h"

#define USRP_CHANNEL        (0)

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
    float symbolrate = 100e3f;
    unsigned int num_packets = 1000;
    unsigned int max_num_attempts = 100;    // maximum number of tx attempts
    unsigned int node_type = NODE_MASTER;
    unsigned int node_id = 15;              // node id in [0,15]
    unsigned int rssi_samples = 128;        // number of samples with which to estimate rssi
    float rssi_clear_threshold = -35.0f;    // rssi threshold to determine if channel is 'clear'
    unsigned int mac_timeout = 5;           // number of 'clear' flags before transmission
    //unsigned int mac_timeout_backoff = 5;   // random backoff
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

    // create usrp object
    usrp_io * usrp = new usrp_io();

    // set properties
    usrp->set_tx_freq(USRP_CHANNEL, frequency);
    usrp->set_tx_samplerate(2*symbolrate);
    usrp->set_rx_freq(USRP_CHANNEL, frequency);
    usrp->set_rx_samplerate(2*symbolrate);
    usrp->enable_auto_tx(USRP_CHANNEL);

    if (verbose) usrp->enable_verbose();
    else         usrp->disable_verbose();

    // initialize iqpr structure
    iqpr q = iqpr_create(node_id,
                         usrp->get_tx_port(USRP_CHANNEL),
                         usrp->get_rx_port(USRP_CHANNEL));

    // sleep for a small time before starting tx/rx processes
    usleep(1000000);

    // start data transfer
    usrp->start_tx(USRP_CHANNEL);
    usrp->start_rx(USRP_CHANNEL);

    // TODO : start timer

    //unsigned int i;
    unsigned int pid;
    unsigned int j;
    unsigned int n;
    unsigned int num_attempts = 0;

    // received frame header
    iqprheader_s header;

    // received frame statistics (SNR, rssi)
    framesyncstats_s stats;

    // parameters
    modulation_scheme ms = MOD_PSK;
    unsigned int bps = 2;
    fec_scheme fec0 = FEC_NONE;
    fec_scheme fec1 = FEC_NONE;


    if (node_type == NODE_MASTER) {
        // 
        // MASTER NODE
        //
        unsigned int payload_len = 1024;
        unsigned char payload[payload_len];

        int ack_received;

        for (pid=0; pid<num_packets; pid++) {

            // initialize payload to random data
            for (n=0; n<payload_len; n++)
                payload[n] = rand() % 256;

            ack_received = 0;
            num_attempts = 0;
            do {
                num_attempts++;

                // wait for clear signal (five clean reports in a row)
                j = mac_timeout;
                while (j) {
                    float rssi = iqpr_mac_getrssi(q,rssi_samples);
                    int clear = rssi < rssi_clear_threshold;
                    if (verbose) printf("  rssi : %12.8f dB %c\n", rssi, clear ? ' ' : '*');

                    if (clear) j--;
                    else       j = mac_timeout;
                }

                // transmit packet
                printf("transmitting packet %6u/%6u (attempt %4u/%4u) %c\n",
                        pid, num_packets, num_attempts, max_num_attempts,
                        num_attempts > 1 ? '*' : ' ');
                iqpr_txpacket(q,pid,payload,payload_len,ms,bps,fec0,fec1);

                // wait for acknowledgement (minimum timeout is about 3)
                for (j=0; j<5; j++) {
                    ack_received = iqpr_wait_for_ack(q, pid, &header, &stats);

                    if (ack_received)
                        break;
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
        unsigned char * payload = NULL;
        unsigned int payload_len;
        unsigned int packet_found;
        pid = 0;

        do {
            // wait for data packet
            do {
                // attempt to receive data packet
                packet_found = iqpr_wait_for_packet(q,
                                                    &payload,
                                                    &payload_len,
                                                    &header,
                                                    &stats);
            } while (!packet_found);

            printf("  ping received %4u data bytes on packet [%4u] : %.2x %.2x %.2x %.2x ...\n",
                    payload_len,
                    header.pid,
                    payload[0],
                    payload[1],
                    payload[2],
                    payload[3]);

            // transmit acknowledgement
            iqpr_txack(q, header.pid);

            pid = header.pid;

        } while (pid != num_packets-1);
    }

    // TODO : stop timer

    // sleep for a small time before stopping tx/rx processes
    usleep(100000);

    // stop data transfer
    usrp->stop_rx(USRP_CHANNEL);
    usrp->stop_tx(USRP_CHANNEL);

    printf("main process complete\n");

    // delete usrp object
    delete usrp;

    // destroy main data object
    iqpr_destroy(q);

    return 0;
}

