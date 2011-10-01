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

#include <iostream>
#include <complex>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <liquid/liquid.h>

#include "usrp_io.h"
 
#define USRP_CHANNEL        (0)
 
static bool verbose;

static unsigned int num_symbols_data=16;// num data symbols
static unsigned int num_symbols_received=0;

static int callback(std::complex<float> * _y,
                    void * _userdata)
{
    num_symbols_received++;
    if (num_symbols_received == num_symbols_data) {
        printf("frame received\n");
        num_symbols_received = 0;
        return 1;
    } else {
        return 0;
    }
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
    // options
    unsigned int num_subcarriers=80;
    unsigned int m=3;
    float beta = 0.9f;
    //modulation_scheme ms = MOD_QAM;
    //unsigned int bps = 2;

    // command-line options
    verbose = true;

    float min_bandwidth = (32e6 * 1.60f / 512.0);
    float max_bandwidth = (32e6 * 1.60f /   4.0);

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

    // compute interpolation rate
    unsigned int decim_rate = (unsigned int)(16e6 * 1.60f / bandwidth);
    
    // ensure multiple of 2
    decim_rate = (decim_rate >> 1) << 1;

    // update actual bandwidth
    bandwidth = 16e6f * 1.60f / (float)(decim_rate);

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

    unsigned int num_blocks = (unsigned int)((4.0f/1.6f*bandwidth*num_seconds)/(512));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_rx_freq(0, frequency);
#if 0
    uio->set_rx_decim(decim_rate);
#else
    uio->set_rx_samplerate(bandwidth);
#endif
    uio->enable_auto_tx(0);

    // retrieve rx port
    gport port_rx = uio->get_rx_port(0);

    // initialize framing descriptors
    unsigned int p[num_subcarriers];    // subcarrier allocation
    unsigned int M_null;                // number of null subcarriers
    unsigned int M_pilot;               // number of pilot subcarriers
    unsigned int M_data;                // number of data subcarriers
    ofdmoqamframe_init_default_sctype(num_subcarriers, p);
#if 0
    // notch part of the spectrum
    for (i=12; i<24; i++)
        p[i] = OFDMOQAMFRAME_SCTYPE_NULL;
#endif
    ofdmoqamframe_validate_sctype(p,
                                  num_subcarriers,
                                  &M_null,
                                  &M_pilot,
                                  &M_data);


    // create frame synchronizer
    ofdmoqamframesync framesync = ofdmoqamframesync_create(num_subcarriers,m,beta,p,callback,NULL);

    // create buffers
    unsigned int rx_buffer_length = 512;
    std::complex<float> data_rx[rx_buffer_length];

    // start usrp data transfer
    uio->start_rx(USRP_CHANNEL);
    printf("usrp data transfer started\n");
 
    unsigned int i;
    num_blocks = 12;
    for (i=0; i<num_blocks; i++) {
        // grab data from port
        gport_consume(port_rx,(void*)data_rx,rx_buffer_length);

        // run through frame synchronizer
        ofdmoqamframesync_execute(framesync, data_rx, rx_buffer_length);
    }
 
    uio->stop_rx(USRP_CHANNEL);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // clean it up
    ofdmoqamframesync_destroy(framesync);

    delete uio;
    return 0;
}

