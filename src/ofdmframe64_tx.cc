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

#define USRP_CHANNEL        (0)

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

    unsigned int num_blocks = (unsigned int)((4.0f/1.6f*bandwidth*num_seconds)/(4096));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_tx_freq(USRP_CHANNEL, frequency);
    uio->set_tx_interp(interp_rate);
    uio->enable_auto_tx(USRP_CHANNEL);

    // retrieve tx port
    gport port_tx = uio->get_tx_port(USRP_CHANNEL);

    resamp2_crcf interpolator = resamp2_crcf_create(37,0.0f,60.0f);

    // framing
    std::complex<float> frame[2048];
    ofdmframe64gen framegen = ofdmframe64gen_create();

    modem mod = modem_create(MOD_QPSK,2);

    // data buffers
    //unsigned char header[24];
    //unsigned char payload[64];
    std::complex<float> syms[48];
    std::complex<float> data_tx[512];

    unsigned int j, n;
    unsigned int i;

    // start usrp data transfer
    uio->start_tx(USRP_CHANNEL);
    printf("usrp data transfer started\n");

    for (i=0; i<num_blocks; i++) {

        //printf("i = %u\n", i);
        // write short/long sequences
        n=0;
        ofdmframe64gen_writeshortsequence(framegen, &frame[n]);
        n+=160;
        ofdmframe64gen_writelongsequence(framegen, &frame[n]);
        n+=160;
        // write symbols
        while (n<2000) {
            // modulate random data
            for (j=0; j<48; j++) {
                unsigned int s = modem_gen_rand_sym(mod);
                modem_modulate(mod,s,&syms[j]);
            }
            ofdmframe64gen_writesymbol(framegen, syms, &frame[n]);
            n += 80;
        }

        // pad end with zeros
        while (n<2048) {
            frame[n++] = 0.0f;
        }

        // send data to usrp_io in blocks
        for (n=0; n<2048; n+=256) {
            // run interpolator
            unsigned int j;
            for (j=0; j<256; j++) {
                resamp2_crcf_interp_execute(interpolator,
                    frame[n+j], &data_tx[2*j]);
            }

            gport_produce(port_tx,(void*)data_tx,512);
        }

        // reset frame generator
        ofdmframe64gen_reset(framegen);
    }
 
 
    uio->stop_tx(USRP_CHANNEL);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // clean it up
    ofdmframe64gen_destroy(framegen);
    resamp2_crcf_destroy(interpolator);
    delete uio;
    return 0;
}

