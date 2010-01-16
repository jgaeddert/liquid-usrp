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
#include <getopt.h>
#include <liquid/liquid.h>

#include "usrp_io.h"

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
    bool verbose = true;

    float min_bandwidth = (32e6 / 512.0);
    float max_bandwidth = (32e6 /   4.0);

    float frequency = 462.5625e6;
    float bandwidth = min_bandwidth;
    float num_seconds = 5.0f;
    
    //
    int d;
    while ((d = getopt(argc,argv,"f:b:s:t:b:qvuh")) != EOF) {
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
    unsigned int interp_rate = (unsigned int)(32e6 / bandwidth);
    
    // ensure multiple of 4
    interp_rate = (interp_rate >> 2) << 2;

    // update actual bandwidth
    bandwidth = 32e6f / (float)(interp_rate);

    if (bandwidth > max_bandwidth) {
        printf("error: maximum bandwidth exceeded (%8.4f MHz)\n", max_bandwidth*1e-6);
        return 0;
    } else if (bandwidth < min_bandwidth) {
        printf("error: minimum bandwidth exceeded (%8.4f kHz)\n", min_bandwidth*1e-3);
        return 0;
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    // 
    unsigned int num_blocks = (unsigned int)((4.0f*bandwidth*num_seconds)/(512));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_tx_freq(0, frequency);
    uio->set_tx_interp(interp_rate);
    uio->enable_auto_tx(0);

    // retrieve tx port from usrp_io object
    gport2 port_tx = uio->get_tx_port(0);
    std::complex<float> data_tx[512];

    // tone parameters
    //  |-------------------|---|       |-------|
    // -pi/2                0  f0       f1     pi/2
    //                        ->|notched|<-
    float f0 = 0.0f;        // lower notching frequency
    float f1 = 0.5f;        // uppper notching frequency
    float f_tone = 0.0f;    // tone frequency
    float df = 0.0001f;     // sweep mode frequency step size
                            // (smaller is faster)
    enum {
        JAMMER_MODE_HOP=0,  // randomly hop frequencies
        JAMMER_MODE_SWEEP   // sweep frequencies
    } mode = JAMMER_MODE_SWEEP;

    // create and initialize the NCO
    //nco nco_tone = nco_create(); // libliquid-0.1.0
    nco nco_tone = nco_create(LIQUID_NCO);
    nco_set_frequency(nco_tone,f_tone);

    unsigned int n; // sample counter
    unsigned int i;

    // start USRP data transfer
    uio->start_tx(0);
    for (i=0; i<num_blocks; i++) {

        // generate tone data (complex sinusoid samples)
        for (n=0; n<512; n++) {
            nco_cexpf(nco_tone, &data_tx[n]);
            nco_step(nco_tone);
        }

        // write data to output buffer
        gport2_produce(port_tx,(void*)data_tx,512);

        // 
        if (mode == JAMMER_MODE_HOP) {
            // random hopping mode: generate random
            // frequency in [-pi/2,pi/2], skipping
            // notched band
            do {
                f_tone = (randf()-0.5f)*M_PI;
            } while (f_tone>f0 && f_tone<f1);

        } else if (mode == JAMMER_MODE_SWEEP) {
            // sweep mode: increment frequency, wrapping
            // around if value is larger than pi/2, and
            // skipping notched band
            f_tone += df;

            if (f_tone>f0 && f_tone<f1)
                f_tone = f1;
            else if (f_tone > M_PI*0.5f)
                f_tone = -M_PI*0.5f;
            else if (f_tone < -M_PI*0.5f)
                f_tone = M_PI*0.5f;
        } else {
            // unknown mode (no frequency change)
        }

        // update the NCO frequency
        nco_set_frequency(nco_tone,f_tone);
    }
 
    uio->stop_tx(0);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // clean up allocated memory objects
    nco_destroy(nco_tone);
    delete uio;
    return 0;
}

