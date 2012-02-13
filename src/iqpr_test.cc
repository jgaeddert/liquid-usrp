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

#include <iostream>
#include <complex>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <time.h>
#include <liquid/liquid.h>

#include "iqpr.h"
 
void usage() {
    printf("iqpr_test:\n");
    printf("  u,h   :   usage/help\n");
}

int main (int argc, char **argv)
{
    //srand(time(NULL));

    //
    int d;
    while ((d = getopt(argc,argv,"uh:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                return 0;
        default:
            fprintf(stderr,"error: %s, unsupported option\n", argv[0]);
            exit(1);
        }
    }

    iqpr q = iqpr_create();

    // set rx parameters
    iqpr_set_rx_gain(q, 20);
    iqpr_set_rx_rate(q, 80e3);
    iqpr_set_rx_freq(q, 462e6f);

    // set tx parameters
    iqpr_set_tx_gain(q, 40);
    iqpr_set_tx_rate(q, 80e3);
    iqpr_set_tx_freq(q, 462e6f);

    // other options
    iqpr_unset_verbose(q);

    unsigned int i;

#if 1
    //
    unsigned int    timespec = 100000;  // microseconds to wait before timing out
    unsigned char * rx_header = NULL;
    int             rx_header_valid;
    unsigned char * rx_payload = NULL;
    unsigned int    rx_payload_len;
    int             rx_payload_valid;
    framesyncstats_s stats;

    //
    // receiver
    //

    printf("starting receiver...\n");
    iqpr_rx_start(q);
    for (i=0; i<200; i++) {
        int packet_received =
        iqpr_rxpacket(q, timespec,
                      &rx_header,
                      &rx_header_valid,
                      &rx_payload,
                      &rx_payload_len,
                      &rx_payload_valid,
                      &stats);

        if (packet_received) {
            printf("iqpr received packet");

            if (rx_header_valid) {
                unsigned int pid = (rx_header[0] << 8) | rx_header[1];
                printf("[%6u]", pid);
            } else {
                printf("header crc : FAIL\n");
            }

            if (rx_payload_valid) {
                printf(" %6u bytes\n", rx_payload_len);
            } else {
                printf("payload crc : FAIL\n");
            }
        } else {
            printf("timed out\n");
        }
    }
    iqpr_rx_stop(q);
#endif

#if 0

    //
    // transmitter
    //

    ofdmflexframegenprops_s fgprops;
    ofdmflexframegenprops_init_default(&fgprops);
    fgprops.check        = LIQUID_CRC_32;
    fgprops.fec0         = LIQUID_FEC_NONE;
    fgprops.fec1         = LIQUID_FEC_NONE;
    fgprops.mod_scheme   = LIQUID_MODEM_QAM4;
#if 0
    fgprops.rampup_len   = 40;
    fgprops.phasing_len  = 40;
    fgprops.rampdn_len   = 40;
#endif

    unsigned int num_packets = 1000;
    unsigned int payload_len = 200;
    unsigned char header[14];
    unsigned char payload[payload_len];

    printf("starting transmitter...\n");
    for (i=0; i<num_packets; i++) {
        //printf("  transmitting packet %6u / %6u\n", i, num_packets);

        header[0] = (i >> 8) & 0xff;
        header[1] = (i    ) & 0xff;

        unsigned int j;
        for (j=2; j<14; j++) header[j] = rand() & 0xff;
        for (j=0; j<payload_len; j++) payload[j] = rand() & 0xff;

        iqpr_txpacket(q, header, payload, payload_len, &fgprops);

        //usleep(10000);
    }
#endif

    // destroy object
    iqpr_destroy(q);

    printf("done.\n");

    return 0;
}

