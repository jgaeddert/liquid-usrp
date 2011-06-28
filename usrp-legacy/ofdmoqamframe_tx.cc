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
#include <getopt.h>
#include <assert.h>
#include <liquid/liquid.h>

#include "usrp_io.h"

#define USRP_CHANNEL        (0)

void usage() {
    printf("ofdmoqamframe_tx:\n");
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
    unsigned int num_subcarriers=80;    // number of subcarriers
    unsigned int num_symbols_S0=7;      // num short sequence symbols
    unsigned int num_symbols_S1=1;      // num long sequence symbols
    unsigned int num_symbols_data=16;   // num data symbols
    unsigned int num_symbols_flush=16;  // num symbols to flush
    unsigned int m=3;
    float beta = 0.9f;
    //modulation_scheme ms = LIQUID_MODEM_QAM;
    //unsigned int bps = 2;

    bool verbose = true;

    float min_bandwidth = (32e6 * 1.60f / 512.0);
    float max_bandwidth = (32e6 * 1.60f /   4.0);

    float frequency = 462.0e6;
    float bandwidth = min_bandwidth;
    float num_seconds = 60.0f;

    unsigned int packet_spacing=8;

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
    unsigned int interp_rate = (unsigned int)(32e6 * 1.60f / bandwidth);
    
    // ensure multiple of 4
    interp_rate = (interp_rate >> 2) << 2;

    // update actual bandwidth
    bandwidth = 32e6f * 1.60f / (float)(interp_rate);

    if (bandwidth > max_bandwidth) {
        printf("error: maximum bandwidth exceeded (%8.4f MHz)\n", max_bandwidth*1e-6);
        return 0;
    } else if (bandwidth < min_bandwidth) {
        printf("error: minimum bandwidth exceeded (%8.4f kHz)\n", min_bandwidth*1e-3);
        return 0;
    } if (packet_spacing < 1) {
        printf("error: packet spacing must be greater than 0\n");
        return 0;
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    unsigned int i;
    unsigned int j;
    unsigned int num_blocks = (unsigned int)((4.0f/1.6f*bandwidth*num_seconds)/(4096));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_tx_freq(USRP_CHANNEL, frequency);
    uio->set_tx_interp(interp_rate);
    uio->enable_auto_tx(USRP_CHANNEL);

    // retrieve tx port
    gport port_tx = uio->get_tx_port(USRP_CHANNEL);

    resamp2_crcf interpolator = resamp2_crcf_create(37,0.0f,60.0f);

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

    // create frame generator
    ofdmoqamframegen framegen = ofdmoqamframegen_create(num_subcarriers,m,beta,p);
    ofdmoqamframegen_print(framegen);

    // framing
    std::complex<float> frame[num_subcarriers];

    modem mod = modem_create(LIQUID_MODEM_QPSK,2);

    // data buffers
    std::complex<float> syms[num_subcarriers];  // data symbols


    // start usrp data transfer
    uio->start_tx(USRP_CHANNEL);
    printf("usrp data transfer started\n");

    num_blocks = 1000000;
    for (i=0; i<num_blocks; i++) {

        //printf("i = %u\n", i);

        // write short sequence(s)
        for (i=0; i<num_symbols_S0; i++) {
            ofdmoqamframegen_writeshortsequence(framegen,frame);
            gport_produce(port_tx,(void*)frame, num_subcarriers);
        }

        // write long sequence
        for (i=0; i<num_symbols_S1; i++) {
            ofdmoqamframegen_writelongsequence(framegen,frame);
            gport_produce(port_tx,(void*)frame, num_subcarriers);
        }

        // modulate data symbols
        for (i=0; i<num_symbols_data; i++) {
        //while (n < 2048) {
            // modulate random data
            for (j=0; j<num_subcarriers; j++) {
                unsigned int s = modem_gen_rand_sym(mod);
                modem_modulate(mod,s,&syms[j]);
            }
            ofdmoqamframegen_writesymbol(framegen, syms, frame);
            gport_produce(port_tx,(void*)frame, num_subcarriers);
        }

        // flush
        for (i=0; i<num_symbols_flush; i++) {
            ofdmoqamframegen_flush(framegen,frame);
            gport_produce(port_tx,(void*)frame, num_subcarriers);
        }

        // reset generator
        ofdmoqamframegen_reset(framegen);
    }
 
 
    uio->stop_tx(USRP_CHANNEL);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // clean it up
    ofdmoqamframegen_destroy(framegen);
    resamp2_crcf_destroy(interpolator);
    delete uio;
    return 0;
}

