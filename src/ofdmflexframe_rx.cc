/*
 * Copyright (c) 2011, 2013 Joseph Gaeddert
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
#include <assert.h>
#include <liquid/liquid.h>

#include <uhd/usrp/multi_usrp.hpp>
 
#include "ofdmtxrx.h"
#include "timer.h"

static bool verbose;

// data counters
unsigned int num_frames_detected;
unsigned int num_valid_headers_received;
unsigned int num_valid_packets_received;
unsigned int num_valid_bytes_received;

// callback function
int callback(unsigned char *  _header,
             int              _header_valid,
             unsigned char *  _payload,
             unsigned int     _payload_len,
             int              _payload_valid,
             framesyncstats_s _stats,
             void *           _userdata)
{
    if (verbose) {
        // compute true carrier offset
        float samplerate = *((float*)_userdata);
        float cfo = _stats.cfo * samplerate / (2*M_PI);
        printf("***** rssi=%7.2fdB evm=%7.2fdB, cfo=%7.3f kHz, ", _stats.rssi, _stats.evm, cfo*1e-3f);

        if (_header_valid) {
            unsigned int packet_id = (_header[0] << 8 | _header[1]);
            printf("rx packet id: %6u", packet_id);
            if (_payload_valid) printf("\n");
            else                printf(" PAYLOAD INVALID\n");
        } else {
            printf("HEADER INVALID\n");
        }
    } else {
    }

    // update global counters
    num_frames_detected++;

    if (_header_valid)
        num_valid_headers_received++;

    if (_payload_valid) {
        num_valid_packets_received++;
        num_valid_bytes_received += _payload_len;
    }

    return 0;
}

void usage() {
    printf("ofdmflexframe_rx -- receive OFDM packets\n");
    printf("  u,h   :   usage/help\n");
    printf("  q/v   :   quiet/verbose\n");
    printf("  f     :   center frequency [Hz], default:  462 MHz\n");
    printf("  b     :   bandwidth [Hz],        default: 1000 kHz\n");
    printf("  G     :   uhd rx gain [dB],      default:   20 dB)\n");
    printf("  A     :   uhd rx antenna\n");
    printf("  M     :   number of subcarriers, default:   48\n");
    printf("  C     :   cyclic prefix length,  default:    6\n");
    printf("  T     :   taper length,          default:    4\n");
    printf("  t     :   run time [seconds],    default:    5\n");
    printf("  d     :   enable debugging mode\n");
}

int main (int argc, char **argv)
{
    // command-line options
    verbose = true;

    float frequency = 462.0e6;
    float bandwidth = 1000e3f;
    float num_seconds = 5.0f;
    float uhd_rxgain = 20.0;
    char uhd_rxantenna[16] = "";        // rx antenna

    // ofdm properties
    unsigned int M = 48;                // number of subcarriers
    unsigned int cp_len = 6;            // cyclic prefix length
    unsigned int taper_len = 4;         // taper length

    int debug_enabled =  0;             // enable debugging?

    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:G:A:M:C:T:t:d")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                            return 0;
        case 'q':   verbose       = false;              break;
        case 'v':   verbose       = true;               break;
        case 'f':   frequency     = atof(optarg);       break;
        case 'b':   bandwidth     = atof(optarg);       break;
        case 'G':   uhd_rxgain    = atof(optarg);       break;
        case 'A':   strncpy(uhd_rxantenna,optarg,15);   break;
        case 'M':   M             = atoi(optarg);       break;
        case 'C':   cp_len        = atoi(optarg);       break;
        case 'T':   taper_len     = atoi(optarg);       break;
        case 't':   num_seconds   = atof(optarg);       break;
        case 'd':   debug_enabled = 1;                  break;
        default:
            usage();
            return 0;
        }
    }

    if (cp_len == 0 || cp_len > M) {
        fprintf(stderr,"error: %s, cyclic prefix must be in (0,M]\n", argv[0]);
        exit(1);
    }

    // create transceiver object
    unsigned char * p = NULL;   // default subcarrier allocation
    ofdmtxrx txcvr(M, cp_len, taper_len, p, callback, (void*)&bandwidth);

    // set properties
    txcvr.set_rx_freq(frequency);
    txcvr.set_rx_rate(bandwidth);
    txcvr.set_rx_gain_uhd(uhd_rxgain);

    // enable debugging on request
    if (debug_enabled)
        txcvr.debug_enable();

    // reset counters
    num_frames_detected=0;
    num_valid_headers_received=0;
    num_valid_packets_received=0;
    num_valid_bytes_received=0;

    // run conditions
    int continue_running = 1;
    timer t0 = timer_create();
    timer_tic(t0);

    // start receiver
    txcvr.start_rx();

    while (continue_running) {
        // sleep for 100 ms and check state
        usleep(100000);

        // check runtime
        if (timer_toc(t0) >= num_seconds)
            continue_running = 0;
    }

    // stop receiver
    printf("ofdmflexframe_rx stopping receiver...\n");
    txcvr.stop_rx();
 
    // compute actual run-time
    float runtime = timer_toc(t0);

    // print results
    float data_rate = num_valid_bytes_received * 8.0f / runtime;
    float percent_headers_valid = (num_frames_detected == 0) ?
                          0.0f :
                          100.0f * (float)num_valid_headers_received / (float)num_frames_detected;
    float percent_packets_valid = (num_frames_detected == 0) ?
                          0.0f :
                          100.0f * (float)num_valid_packets_received / (float)num_frames_detected;
    printf("    frames detected     : %6u\n", num_frames_detected);
    printf("    valid headers       : %6u (%6.2f%%)\n", num_valid_headers_received,percent_headers_valid);
    printf("    valid packets       : %6u (%6.2f%%)\n", num_valid_packets_received,percent_packets_valid);
    printf("    bytes received      : %6u\n", num_valid_bytes_received);
    printf("    run time            : %f s\n", runtime);
    printf("    data rate           : %8.4f kbps\n", data_rate*1e-3f);

    // destroy objects
    timer_destroy(t0);

    return 0;
}

