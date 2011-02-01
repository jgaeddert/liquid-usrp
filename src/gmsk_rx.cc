/*
 * Copyright (c) 2011 Joseph Gaeddert
 * Copyright (c) 2011 Virginia Polytechnic Institute & State University
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
#include <liquid/liquid.h>

#include "usrp_io.h"
 
#define USRP_CHANNEL        (0)
 
static bool verbose;

static unsigned int num_packets_received;
static unsigned int num_valid_packets_received;
static unsigned int num_valid_bytes_received;

int callback(unsigned char * _payload,
             int _payload_valid,
             unsigned int _payload_len,
             void * _userdata)
{

   num_packets_received++;
   if ( !_payload_valid ) {
        if (verbose) printf("payload crc : FAIL\n");
    } else {
        num_valid_packets_received++;
        num_valid_bytes_received += _payload_len;
    }
    return 0;
}

void usage() {
    printf("gmsk_rx:\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  s     :   symbol rate [Hz], default: 62.5 kHz\n");
    printf("  B     :   bandwidth factor, default: 0.3\n");
    printf("  t     :   run time [seconds]\n");
    printf("  q     :   quiet\n");
    printf("  v     :   verbose\n");
    printf("  u,h   :   usage/help\n");
}

int main (int argc, char **argv)
{
    // command-line options
    verbose = true;

    float min_symbol_rate = (32e6 / 512.0);
    float max_symbol_rate = (32e6 /   4.0);

    float frequency = 462.0e6;
    float symbol_rate = min_symbol_rate;
    float num_seconds = 5.0f;

    // demodulator properties
    unsigned int k=2;
    unsigned int m=3;
    float BT = 0.3f;

    //
    int d;
    while ((d = getopt(argc,argv,"f:s:B:t:qvuh")) != EOF) {
        switch (d) {
        case 'f':   frequency = atof(optarg);       break;
        case 's':   symbol_rate = atof(optarg);     break;
        case 'B':   BT = atof(optarg);              break;
        case 't':   num_seconds = atof(optarg);     break;
        case 'q':   verbose = false;                break;
        case 'v':   verbose = true;                 break;
        case 'u':
        case 'h':
        default:
            usage();
            return 0;
        }
    }

    if (symbol_rate > max_symbol_rate) {
        printf("error: maximum symbol_rate exceeded (%8.4f MHz)\n", max_symbol_rate*1e-6);
        return 0;
    } else if (symbol_rate < min_symbol_rate) {
        printf("error: minimum symbol_rate exceeded (%8.4f kHz)\n", min_symbol_rate*1e-3);
        return 0;
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("symbol_rate   :   %12.8f [kHz]\n", symbol_rate*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    unsigned int rx_buffer_length = 128*k; // must be multiple of k
    unsigned int num_blocks = (unsigned int)((2.0f*symbol_rate*num_seconds)/(rx_buffer_length));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_rx_freq(USRP_CHANNEL, frequency);
    uio->set_rx_samplerate(2.0f*symbol_rate);
    uio->enable_auto_tx(USRP_CHANNEL);

    // retrieve rx port
    gport port_rx = uio->get_rx_port(USRP_CHANNEL);

    // create synchronization objects
    agc_crcf agc_rx = agc_crcf_create();
    gmskdem demod = gmskdem_create(k, m, BT);
    bpacketsync ps = bpacketsync_create(0, callback, NULL);

    // set object properties
    agc_crcf_set_target(agc_rx, 1.0f);
    agc_crcf_set_bandwidth(agc_rx, 1e-3f);
    agc_crcf_set_gain_limits(agc_rx, 1e-3f, 1e4f);
    gmskdem_set_bw(demod, 0.1f);
 
    std::complex<float> data_rx[rx_buffer_length];
    std::complex<float> sym_rx[k];

    // start data transfer
    uio->start_rx(USRP_CHANNEL);
    // consume first few blocks to allow hardware to settle
    gport_consume(port_rx,(void*)data_rx,rx_buffer_length);
    printf("usrp data transfer started\n");
 
    // reset counter
    num_packets_received = 0;
    num_valid_packets_received = 0;
    num_valid_bytes_received = 0;

    unsigned int i;
    unsigned int n;
    unsigned int num_samples=0;
    unsigned int num_bits=0;
    unsigned char byte=0;
    for (n=0; n<num_blocks; n++) {
        // grab data from port
        gport_consume(port_rx,(void*)data_rx,rx_buffer_length);

        // 
        // run through synchronizer
        //
        for (i=0; i<rx_buffer_length; i++) {
            // automatic gain control
            std::complex<float> agc_out;
            agc_crcf_execute(agc_rx, data_rx[i], &agc_out);

            // add sample to symbol buffer
            sym_rx[num_samples] = agc_out;
            num_samples++;
            if (num_samples == k) {
                num_samples = 0;

                // push through gmsk demodulator
                unsigned int demod_sym;
                gmskdem_demodulate(demod, sym_rx, &demod_sym);

                // append demodulated bit to byte
                byte <<= 1;
                byte |= demod_sym & 0x01;
                num_bits++;
                if (num_bits >= 8) {
                    bpacketsync_execute(ps, byte);
                    byte = 0;
                    num_bits = 0;
                }

            }
        }

    }
    gmskdem_print(demod);
 
    uio->stop_rx(USRP_CHANNEL);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // print results
    float data_rate = num_valid_bytes_received * 8.0f / num_seconds;
    float percent_valid = (num_packets_received == 0) ?
                          0.0f :
                          100.0f * (float)num_valid_packets_received / (float)num_packets_received;
    printf("    packets received    : %6u\n", num_packets_received);
    printf("    valid packets       : %6u (%6.2f%%)\n", num_valid_packets_received,percent_valid);
    printf("    data rate           : %8.4f kbps\n", data_rate*1e-3f);

    // destroy synchronization objects
    agc_crcf_destroy(agc_rx);
    gmskdem_destroy(demod);
    bpacketsync_destroy(ps);

    delete uio;
    return 0;
}

