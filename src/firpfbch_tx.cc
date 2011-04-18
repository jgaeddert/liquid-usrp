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
// firpfbch_tx  : finite impulse response polyphase filterbank
//                channelizer (transmitter)
//

#include <iostream>
#include <complex>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <liquid/liquid.h>

#include "usrp_io.h"
 
#define USRP_CHANNEL        (0)

void usage() {
    printf("packet_tx:\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  s     :   symbol rate [Hz] (62.5kHz min, 8MHz max)\n");
    printf("  t     :   run time [seconds]\n");
    printf("  m     :   filter delay [symbols]\n");
    printf("  b     :   filter excess bandwidth factor [0.0 min, 1.0 max]\n");
    printf("  q     :   quiet\n");
    printf("  v     :   verbose\n");
    printf("  u,h   :   usage/help\n");
}

int main (int argc, char **argv)
{
    // command-line options
    bool verbose = true;

    float min_symbol_rate = (32e6 / 512.0);
    float max_symbol_rate = (32e6 /   4.0);

    float frequency = 462.0e6;
    float symbol_rate = min_symbol_rate;
    float num_seconds = 5.0f;
    unsigned int m = 3;
    float beta = 0.3f;


    //
    int d;
    while ((d = getopt(argc,argv,"f:s:t:m:b:qvuh")) != EOF) {
        switch (d) {
        case 'f':   frequency = atof(optarg);       break;
        case 's':   symbol_rate = atof(optarg);     break;
        case 't':   num_seconds = atof(optarg);     break;
        case 'm':   m = atoi(optarg);               break;
        case 'b':   beta = atof(optarg);            break;
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
    unsigned int interp_rate = (unsigned int)(32e6 / symbol_rate);
    
    // ensure multiple of 4
    interp_rate = (interp_rate >> 2) << 2;

    // update actual symbol_rate
    symbol_rate = 32e6f / (float)(interp_rate);

    if (symbol_rate > max_symbol_rate) {
        printf("error: maximum symbol_rate exceeded (%8.4f MHz)\n", max_symbol_rate*1e-6);
        return 0;
    } else if (symbol_rate < min_symbol_rate) {
        printf("error: minimum symbol_rate exceeded (%8.4f kHz)\n", min_symbol_rate*1e-3);
        return 0;
    } else if (m < 1 || m > 20) {
        printf("error: filter length m must be in [1,20]\n");
        return 0;
    } else if (beta < 0.0f || beta > 1.0f) {
        printf("error: filter excess bandwidth beta must be in [0.0,1.0]\n");
        return 0;
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("symbol_rate :   %12.8f [kHz]\n", symbol_rate*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    // 
    unsigned int num_blocks = (unsigned int)((4.0f*symbol_rate*num_seconds)/(512));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_tx_freq(USRP_CHANNEL, frequency);
    uio->set_tx_interp(interp_rate);
    uio->enable_auto_tx(USRP_CHANNEL);

    // retrieve tx port from usrp_io object
    gport port_tx = uio->get_tx_port(USRP_CHANNEL);

    // parameters
    //unsigned int k=2;   // samples/symbol
    unsigned int num_channels=64;
    //unsigned int m=3;   // symbol delay
    //float dt=0.0f;      // fractional sample delay
    //float beta=0.3f;    // excess bandwidth factor

    // create half-band interpolator
    resamp2_crcf interpolator = resamp2_crcf_create(37,0.0f,60.0f);
    std::complex<float> data_tx[2*num_channels];

    // 
    std::complex<float> symbols[num_channels];
    std::complex<float> synthesizer_out[num_channels];

    // modem
    modulation_scheme ms = LIQUID_MODEM_QPSK;
    unsigned int bps = 2;
    modem mod = modem_create(ms,bps);

    // create channelizer
    firpfbch cs = firpfbch_create(num_channels, 3, -60.0f, 0, FIRPFBCH_NYQUIST, FIRPFBCH_SYNTHESIZER);

    unsigned int n; // sample counter
    unsigned int s; // data symbol
    
    unsigned int i;

    // start USRP data transfer
    uio->start_tx(USRP_CHANNEL);
    for (i=0; i<num_blocks; i++) {

        // generate data symbols
        for (n=0; n<num_channels; n++) {
            s = modem_gen_rand_sym(mod);
            modem_modulate(mod,s,&symbols[n]);

            // disable guard channels
            if (n > 26 && n < 38)
                symbols[n] = 0.0f;

            // disable notched channels
            if (n > 12 && n < 16)
                symbols[n] = 0.0f;
        }

        // run channelizer
        firpfbch_synthesizer_execute(cs,symbols,synthesizer_out);

        // run half-band interpolator
        for (n=0; n<num_channels; n++)
            resamp2_crcf_interp_execute(interpolator,synthesizer_out[n],&data_tx[2*n]);

        gport_produce(port_tx,(void*)data_tx,2*num_channels);

    }
 
    uio->stop_tx(USRP_CHANNEL);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // clean it up
    resamp2_crcf_destroy(interpolator);
    modem_destroy(mod);
    firpfbch_destroy(cs);
    delete uio;
    return 0;
}

