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
#include <liquid/liquid.h>

#include "usrp_io.h"
 
#define USRP_CHANNEL        (0)

void usage() {
    printf("packet_tx:\n");
    printf("  f     :   center frequency [Hz] (default: 462 MHz)\n");
    printf("  b     :   bandwidth [Hz], [62.5kHz, 8MHz] (default: 62.5kHz)\n");
    printf("  p     :   packet spacing (default: 1)\n");
    printf("  g     :   transmit power gain [dB] (default: -3dB)\n");
    printf("  t     :   run time [seconds] (default: 5.0)\n");
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

    float frequency = 462.0e6;
    float bandwidth = min_bandwidth;
    float num_seconds = 5.0f;
    float txgain_dB = -3.0f;

    unsigned int packet_spacing=1;

    //
    int d;
    while ((d = getopt(argc,argv,"f:b:p:g:t:qvuh")) != EOF) {
        switch (d) {
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 'p':   packet_spacing = atoi(optarg);  break;
        case 'g':   txgain_dB = atof(optarg);       break;
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
    printf("tx gain     :   %12.8f [dB]\n", txgain_dB);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    unsigned int num_blocks = (unsigned int)((4.0f*bandwidth*num_seconds)/(4096));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_tx_freq(USRP_CHANNEL, frequency);
    uio->set_tx_samplerate(2.0f*bandwidth);
    uio->enable_auto_tx(USRP_CHANNEL);

    // retrieve tx port
    gport port_tx = uio->get_tx_port(USRP_CHANNEL);

    // framegen parameters
    //unsigned int k=2; // samples per symbol
    unsigned int m=3; // delay
    float beta=0.7f;  // excess bandwidth factor

    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/10.0f);
 
    // framing
    std::complex<float> frame[2048];
    framegen64 framegen = framegen64_create(m,beta);

    // data buffers
    unsigned char header[24];
    unsigned char payload[64];

    unsigned int j, pid=0;

    // start usrp data transfer
    uio->start_tx(USRP_CHANNEL);

    unsigned int i;
    for (i=0; i<num_blocks; i++) {
        // generate the frame / transmit silence
        if ((i%(packet_spacing+1))==0) {
            // generate random data
            for (j=0; j<24; j++)    header[j]  = rand() % 256;
            for (j=0; j<64; j++)    payload[j] = rand() % 256;
            header[0] = (pid >> 8) & 0x00ff;
            header[1] = (pid     ) & 0x00ff;
            if (verbose)
                printf("packet id: %u\n", pid);
            pid = (pid+1) & 0xffff;

            framegen64_execute(framegen, header, payload, frame);
        } else {
            framegen64_flush(framegen, 2048, frame);
        }

        // apply gain
        for (j=0; j<2048; j++)
            frame[j] *= g;

        gport_produce(port_tx,(void*)frame,2048);

    }
 
    uio->stop_tx(USRP_CHANNEL);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // clean it up
    delete uio;
    return 0;
}

