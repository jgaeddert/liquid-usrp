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
 
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <complex>
#include <getopt.h>
#include <liquid/liquid.h>

#include "usrp_io.h"
 
#define USRP_CHANNEL        (0)

void usage() {
    printf("ofdmframe_tx -- transmit OFDM packets\n");
    printf("  u,h   :   usage/help\n");
    printf("  q/v   :   quiet/verbose\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  b     :   bandwidth [Hz] (62.5kHz min, 8MHz max)\n");
    printf("  M     :   number of subcarriers, default: 64\n");
}

int main (int argc, char **argv)
{
    // command-line options
    bool verbose = true;

    float min_bandwidth = (32e6 / 512.0);
    float max_bandwidth = (32e6 /   4.0);

    float frequency = 462.0e6;
    float bandwidth = min_bandwidth;
    //float num_seconds = 5.0f;

    unsigned int M = 64;                // number of subcarriers
    unsigned int cp_len = 16;           // cyclic prefix length
    unsigned int num_symbols_S0 = 2;    // number of S0 symbols
    unsigned int num_symbols_S1 = 2;    // number of S0 symbols
    unsigned int num_symbols_data = 8;  // number of data symbols
    modulation_scheme ms = MOD_QAM;
    unsigned int bps = 4;

    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:M:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose = false;                break;
        case 'v':   verbose = true;                 break;
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 'M':   M = atoi(optarg);               break;
        default:
            usage();
            return 0;
        }
    }

    if (bandwidth > max_bandwidth) {
        fprintf(stderr,"error: %s, maximum bandwidth exceeded (%8.4f MHz)\n", argv[0], max_bandwidth*1e-6);
        return 0;
    } else if (bandwidth < min_bandwidth) {
        fprintf(stderr,"error: %s, minimum bandwidth exceeded (%8.4f kHz)\n", argv[0], min_bandwidth*1e-3);
        exit(1);
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    // 
    //unsigned int num_blocks = (unsigned int)((4.0f*bandwidth*num_seconds)/(512));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_tx_freq(0, frequency);
    uio->set_tx_samplerate(2.0f*bandwidth);
    uio->enable_auto_tx(0);

    // retrieve tx port from usrp_io object
    gport port_tx = uio->get_tx_port(0);


    // initialize subcarrier allocation
    unsigned int p[M];
    ofdmframe_init_default_sctype(M, p);

    // create frame generator
    ofdmframegen fg = ofdmframegen_create(M, cp_len, p);
    ofdmframegen_print(fg);

    // arrays
    std::complex<float> X[M];           // channelized symbols
    std::complex<float> y[M+cp_len];    // output time series

    // create modem
    modem mod = modem_create(ms,bps);

    unsigned int i;
    unsigned int j;
    unsigned int n;

    // start USRP data transfer
    uio->start_tx(0);
    unsigned int num_frames = -1;
    for (i=0; i<num_frames; i++) {

        // write short sequence(s)
        for (n=0; n<num_symbols_S0; n++) {
            ofdmframegen_write_S0(fg, y);
            gport_produce(port_tx, (void*)y, M);
        }

        // write long sequence(s)
        for (n=0; n<num_symbols_S1; n++) {
            ofdmframegen_write_S1(fg, y);
            gport_produce(port_tx, (void*)y, M);
        }

        // generate random symbols
        for (n=0; n<num_symbols_data; n++) {
            for (j=0; j<M; j++) {
                unsigned int s = modem_gen_rand_sym(mod);
                modem_modulate(mod,s,&X[j]);
            }

            ofdmframegen_writesymbol(fg, X, y);

            gport_produce(port_tx, (void*)y, M+cp_len);
        }

        // flush with zeros
        //for (n=0; n<M+cp_len; n++)
    }
 
    uio->stop_tx(0);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // clean it up
    ofdmframegen_destroy(fg);
    modem_destroy(mod);
    delete uio;
    return 0;
}

