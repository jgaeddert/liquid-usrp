 // Simple C++ USRP interfacing demonstration program
 //
 //
 // This program was derived and modified from test_usrp_standard_tx.cc 
 
 /* -*- c++ -*- */
 /*
  * Copyright 2003,2006,2007,2008 Free Software Foundation, Inc.
  * 
  * This file is part of GNU Radio
  * 
  * GNU Radio is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 3, or (at your option)
  * any later version.
  * 
  * GNU Radio is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  * 
  * You should have received a copy of the GNU General Public License
  * along with GNU Radio; see the file COPYING.  If not, write to
  * the Free Software Foundation, Inc., 51 Franklin Street,
  * Boston, MA 02110-1301, USA.
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
    float beta = 0.3f;


    //
    int d;
    while ((d = getopt(argc,argv,"f:s:t:b:qvuh")) != EOF) {
        switch (d) {
        case 'f':   frequency = atof(optarg);       break;
        case 's':   symbol_rate = atof(optarg);     break;
        case 't':   num_seconds = atof(optarg);     break;
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
    uio->set_tx_freq(0, frequency);
    uio->set_tx_interp(interp_rate);
    uio->enable_auto_tx(0);

    // retrieve tx port from usrp_io object
    gport port_tx = uio->get_tx_port(0);
    std::complex<float> * data_tx;

    // frequency modulator
    float m = 0.01f;
    float f = 0.0f;
    freqmodem fm = freqmodem_create(m,f);

    // audio NCO
    float f_audio = 0.01f;
    nco nco_audio = nco_create(LIQUID_NCO);
    nco_set_frequency(nco_audio,f_audio);

    unsigned int n; // sample counter
    unsigned int i;

    // start USRP data transfer
    uio->start_tx(0);
    for (i=0; i<num_blocks; i++) {

        // retrieve tx data buffer
        data_tx = (std::complex<float>*) gport_producer_lock(port_tx,512);

        // generate FM data
        for (n=0; n<512; n++) {
            freqmodem_modulate(fm, nco_cos(nco_audio), &data_tx[n]);
            nco_step(nco_audio);
        }

        gport_producer_unlock(port_tx,512);

    }
 
    uio->stop_tx(0);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // clean it up
    freqmodem_destroy(fm);
    nco_destroy(nco_audio);
    delete uio;
    return 0;
}

