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
    printf("  m/s   :   designate node as master/slave, default: slave\n");
}

int main (int argc, char **argv) {
    // options
    float frequency = 462e6f;
    float symbolrate = 100e3f;
    unsigned int num_packets = 1000;
    unsigned int max_num_attempts = 100;    // maximum number of tx attempts
    unsigned int node_type = NODE_MASTER;
    unsigned int node_id = 100;

    //
    int d;
    while ((d = getopt(argc,argv,"uhf:b:n:ms")) != EOF) {
        switch (d) {
        case 'u':
        case 'h': usage();                      return 0;
        case 'f': frequency = atof(optarg);     break;
        case 'b': symbolrate = atof(optarg);    break;
        case 'n': num_packets = atoi(optarg);   break;
        case 'm': node_type = NODE_MASTER;      break;
        case 's': node_type = NODE_SLAVE;       break;
        default:
            fprintf(stderr,"error: %s, unsupported option\n", argv[0]);
            exit(1);
        }
    }

    // initialize iqpr structure
    iqpr q = iqpr_create(node_id);

    // create usrp object
    usrp_io * usrp = new usrp_io();

    // set properties
    usrp->set_tx_freq(USRP_CHANNEL, frequency);
    usrp->set_tx_samplerate(2*symbolrate);
    usrp->set_rx_freq(USRP_CHANNEL, frequency);
    usrp->set_rx_samplerate(2*symbolrate);
    usrp->enable_auto_tx(USRP_CHANNEL);

    // initialize ports
    iqpr_connect_txport(q, usrp->get_tx_port(USRP_CHANNEL));
    iqpr_connect_rxport(q, usrp->get_rx_port(USRP_CHANNEL));

    usleep(1000000);

    // start data transfer
    usrp->start_tx(USRP_CHANNEL);
    usrp->start_rx(USRP_CHANNEL);

    // TODO : start timer

    unsigned int i;
    unsigned int j;
    unsigned int n;
    unsigned int num_attempts = 0;
    if (node_type == NODE_MASTER) {
        unsigned int payload_len = 1024;
        unsigned char payload[payload_len];

        // initialize payload to random data
        for (n=0; n<payload_len; n++)
            payload[n] = rand() % 256;

        int ack_received;

        for (i=0; i<num_packets; i++) {

            ack_received = 0;
            num_attempts = 0;
            do {
                num_attempts++;

                // wait for clear signal (five clean reports in a row)
                j = 5;
                while (j) {
                    if (iqpr_mac_clear(q))  j--;
                    else                    j = 5;
                }

                // transmit packet
                printf("transmitting packet %6u (attempt %3u)\n", i, num_attempts);
                iqpr_txpacket(q,i,payload,payload_len);

                // wait for acknowledgement (minimum timeout is about 3)
                for (j=0; j<5; j++) {
                    if (iqpr_wait_for_ack(q,i)) {
                        ack_received = 1;
                        break;
                    }
                }
            } while (!ack_received && (num_attempts < max_num_attempts) );

            if (num_attempts == max_num_attempts) {
                printf("transmitter reached maximum number of attemts; bailing\n");
                break;
            }
        }
    } else {
        unsigned char * payload = NULL;
        unsigned int payload_len;
        int pid;
        for (i=0; i<1000; i++) {
            // wait for data packet
            pid = -1;
            do {
                pid = iqpr_wait_for_data(q,&payload,&payload_len);
            } while ( pid == -1 );

            printf("  ping received %4u data samples on packet [%4u] : %.2x %.2x %.2x %.2x\n",
                    payload_len, pid,
                    payload[0],
                    payload[1],
                    payload[2],
                    payload[3]);

            // transmit acknowledgement
            iqpr_txack(q,pid);
        }
    }

    // TODO : stop timer

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

