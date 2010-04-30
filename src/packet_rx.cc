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

#include <math.h>
#include <iostream>
#include <complex>
#include <liquid/liquid.h>

#include "usrp_io.h"
 
#define USRP_CHANNEL        (0)
 
static bool verbose;

static unsigned int num_packets_received;
static unsigned int num_valid_packets_received;

static int callback(unsigned char * _header,  int _header_valid,
                    unsigned char * _payload, int _payload_valid,
                    void * _userdata)
{

    if (verbose) printf("********* callback invoked, ");
    num_packets_received++;

    if ( !_header_valid ) {
        if (verbose) printf("header crc : FAIL\n");
    } else if ( !_payload_valid ) {
        if (verbose) printf("payload crc : FAIL\n");
    } else {
        if (verbose) {
            unsigned int pid = (_header[0] << 8) | _header[1];
            printf("packet id: %u\n", pid);
        }
        num_valid_packets_received++;
    }
    return 0;
}

void usage() {
    printf("packet_tx:\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  b     :   bandwidth [Hz]\n");
    printf("  t     :   run time [seconds]\n");
    printf("  q     :   quiet\n");
    printf("  v     :   verbose\n");
    printf("  u,h   :   usage/help\n");
}

int main (int argc, char **argv)
{
    // command-line options
    verbose = true;

    float min_bandwidth = (32e6 / 512.0);
    float max_bandwidth = (32e6 /   4.0);

    float frequency = 462.0e6;
    float bandwidth = min_bandwidth;
    float num_seconds = 5.0f;

    //
    int d;
    while ((d = getopt(argc,argv,"f:b:t:qvuh")) != EOF) {
        switch (d) {
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
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

    if (bandwidth > max_bandwidth) {
        printf("error: maximum bandwidth exceeded (%8.4f MHz)\n", max_bandwidth*1e-6);
        return 0;
    } else if (bandwidth < min_bandwidth) {
        printf("error: minimum bandwidth exceeded (%8.4f kHz)\n", min_bandwidth*1e-3);
        return 0;
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    unsigned int num_blocks = (unsigned int)((4.0f*bandwidth*num_seconds)/(512));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_rx_freq(USRP_CHANNEL, frequency);
    uio->set_rx_samplerate(2.0f*bandwidth);
    uio->enable_auto_tx(USRP_CHANNEL);

    // retrieve rx port
    gport port_rx = uio->get_rx_port(USRP_CHANNEL);

    // framing
    unsigned int m=3;
    float beta=0.7f;
    framesync64 framesync = framesync64_create(m,beta,callback,NULL);

    // set properties
    framesync64_set_squelch_threshold(framesync, -28.0f);   // RSSI threshold

    // create decimator
    resamp2_crcf decimator = resamp2_crcf_create(37,0.0f,60.0f);
    
    unsigned int rx_buffer_length = 512;
    std::complex<float> decim_out[rx_buffer_length];
 
    // start data transfer
    uio->start_rx(USRP_CHANNEL);
    printf("usrp data transfer started\n");
 
    unsigned int n;
    std::complex<float> data_rx[rx_buffer_length];

    // reset counter
    num_packets_received = 0;
    num_valid_packets_received = 0;

    unsigned int i;
    for (i=0; i<num_blocks; i++) {
        // grab data from port
        gport_consume(port_rx,(void*)data_rx,rx_buffer_length);

        // run decimator
        for (n=0; n<rx_buffer_length/2; n++) {
            resamp2_crcf_decim_execute(decimator, &data_rx[2*n], &decim_out[n]);
        }

        // run through frame synchronizer
        framesync64_execute(framesync, decim_out, rx_buffer_length/2);
    }
 
 
    uio->stop_rx(USRP_CHANNEL);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // print results
    float data_rate = num_valid_packets_received * 8.0f * 64.0f / num_seconds;
    float percent_valid = (num_packets_received == 0) ?
                          0.0f :
                          100.0f * (float)num_valid_packets_received / (float)num_packets_received;
    printf("    packets received    : %6u\n", num_packets_received);
    printf("    valid packets       : %6u (%6.2f%%)\n", num_valid_packets_received,percent_valid);
    printf("    data rate           : %12.8f kbps\n", data_rate*1e-3f);

    // clean it up
    framesync64_destroy(framesync);
    resamp2_crcf_destroy(decimator);

    delete uio;
    return 0;
}

