 // Simple C++ USRP interfacing demonstration program
 //
 //
 // This program was derived and modified from test_usrp_standard_rx.cc 
 
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
#include <liquid/liquid.h>

#include "usrp_io.h"
 
/*
 SAMPLES_PER_READ :Each sample is consists of 4 bytes (2 bytes for I and 
 2 bytes for Q. Since the reading length from USRP should be multiple of 512 
 bytes see "usrp_basic.h", then we have to read multiple of 128 samples each 
 time (4 bytes * 128 sample = 512 bytes)  
 */
#define SAMPLES_PER_READ    (512)       // Must be a multiple of 128
#define USRP_CHANNEL        (0)
 
static bool verbose;

static int callback(unsigned char * _header,  int _header_valid,
                    unsigned char * _payload, int _payload_valid,
                    void * _userdata)
{
    if (!verbose)
        return 0;

    std::cout << "********* callback invoked, ";// << std::endl;
    if ( !_header_valid ) {
        printf("header crc : FAIL\n");
    } else if ( !_payload_valid ) {
        printf("payload crc : FAIL\n");
    } else {
        printf("packet id: %u\n", (unsigned int ) _header[0]);
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
    int    total_reads = 200;
    int    i;

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
        case 'f':
            frequency = atof(optarg);
            break;
        case 'b':
            bandwidth = atof(optarg);
            break;
        case 't':
            num_seconds = atof(optarg);
            break;
        case 'q':
            verbose = false;
            break;
        case 'v':
            verbose = true;
            break;
        case 'u':
        case 'h':
        default:
            usage();
            return 0;
        }
    }

    // compute interpolation rate
    unsigned int decim_rate = (unsigned int)(16e6 / bandwidth);
    
    // ensure multiple of 2
    decim_rate = (decim_rate >> 1) << 1;

    // update actual bandwidth
    bandwidth = 16e6f / (float)(decim_rate);

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

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_rx_freq(0, frequency);
    uio->set_rx_decim(decim_rate);
    uio->enable_auto_tx(0);

    // retrieve rx port
    gport port_rx = uio->get_rx_port(0);

    // framing
    unsigned int m=3;
    float beta=0.7f;
    framesync64 framesync = framesync64_create(m,beta,callback,NULL);

    // create decimator
    resamp2_crcf decimator = resamp2_crcf_create(37,0.0f,60.0f);
    
    unsigned int rx_buffer_length = 512;
    std::complex<float> decim_out[rx_buffer_length];
 
    // start data transfer
    uio->start_rx(0);
    printf("usrp data transfer started\n");
 
    unsigned int n;
    std::complex<float> * data_rx;

    for (i = 0; i < total_reads; i++) {
        // grab data from port
        data_rx = (std::complex<float>*) gport_consumer_lock(port_rx,rx_buffer_length);

        // run decimator
        for (n=0; n<rx_buffer_length/2; n++) {
            resamp2_crcf_decim_execute(decimator, &data_rx[2*n], &decim_out[n]);
        }

        // run through frame synchronizer
        framesync64_execute(framesync, decim_out, rx_buffer_length/2);

        gport_consumer_unlock(port_rx,rx_buffer_length);
    }
 
 
    uio->stop_rx(0);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // clean it up
    framesync64_destroy(framesync);
    resamp2_crcf_destroy(decimator);

    delete uio;
    return 0;
}

