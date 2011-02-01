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

#include <iostream>
#include <complex>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <liquid/liquid.h>

#include "usrp_io.h"
 
#define USRP_CHANNEL        (0)
#define DEBUG               (0)
 
static bool verbose;

static unsigned int num_packets_received;
static unsigned int num_valid_packets_received;
static unsigned int num_valid_bytes_received;

int callback(unsigned char * _payload,
             int _payload_valid,
             unsigned int _payload_len,
             void * _userdata)
{

   num_packets_received++;
   if ( !_payload_valid ) {
        if (verbose) printf("payload crc : FAIL\n");
    } else {
        num_valid_packets_received++;
        num_valid_bytes_received += _payload_len;
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

    unsigned int rx_buffer_length = 512;
    unsigned int num_blocks = (unsigned int)((2.0f*bandwidth*num_seconds)/(rx_buffer_length));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_rx_freq(USRP_CHANNEL, frequency);
    uio->set_rx_samplerate(2.0f*bandwidth);
    uio->enable_auto_tx(USRP_CHANNEL);

    // retrieve rx port
    gport port_rx = uio->get_rx_port(USRP_CHANNEL);

    // 
    unsigned int npfb = 32;
    unsigned int m=3;
    float beta=0.7f;
    unsigned int H_len = 2*2*npfb*m + 1;
    float H[H_len];
    design_rrc_filter(2*npfb,m,beta,0,H);

    // create synchronization objects
    agc_crcf agc_rx = agc_crcf_create();
    nco_crcf nco_rx = nco_crcf_create(LIQUID_VCO);
    symsync_crcf mfdecim = symsync_crcf_create(2, npfb, H, H_len-1);
    modem demod = modem_create(MOD_DPSK, 2);
    bpacketsync ps = bpacketsync_create(0, callback, NULL);

    // set object properties
    agc_crcf_set_target(agc_rx, 1.0f);
    agc_crcf_set_bandwidth(agc_rx, 1e-3f);
    agc_crcf_set_gain_limits(agc_rx, 1e-3f, 1e4f);
    symsync_crcf_set_lf_bw(mfdecim, 0.02f);
    nco_crcf_pll_set_bandwidth(nco_rx, 0.08f);
 
    std::complex<float> data_rx[rx_buffer_length];

    // start data transfer
    uio->start_rx(USRP_CHANNEL);
    // consume first few blocks to allow hardware to settle
    gport_consume(port_rx,(void*)data_rx,rx_buffer_length);
    printf("usrp data transfer started\n");
 
    // reset counter
    num_packets_received = 0;
    num_valid_packets_received = 0;
    num_valid_bytes_received = 0;

    unsigned int i;
    unsigned int n;
    unsigned int num_bits=0;
    unsigned char byte=0;
#if DEBUG
    FILE * fid = fopen("rx_rrc_debug.m","w");
    fprintf(fid,"%% : auto-generated file\n");
    fprintf(fid,"clear all\n");
    fprintf(fid,"close all\n");
    int file_open = 1;
    unsigned int k=0;
#endif
    for (n=0; n<num_blocks; n++) {
        // grab data from port
        gport_consume(port_rx,(void*)data_rx,rx_buffer_length);

        // 
        // run through synchronizer
        //
        for (i=0; i<rx_buffer_length; i++) {
            // automatic gain control
            std::complex<float> agc_out;
            agc_crcf_execute(agc_rx, data_rx[i], &agc_out);
            //agc_out = data_rx[i] * 2.0f;

            // symbol timing recovery
            std::complex<float> mfdecim_out[4];
            unsigned int nw;
            symsync_crcf_execute(mfdecim, &agc_out, 1, mfdecim_out, &nw);

            unsigned int j;
            for (j=0; j<nw; j++) {
                // carrier phase recovery
                std::complex <float> nco_out;
                nco_crcf_mix_down(nco_rx, mfdecim_out[j], &nco_out);

                // demodulate and retrieve phase error
                unsigned int demod_sym;
                float phi;
                modem_demodulate(demod, nco_out, &demod_sym);
                get_demodulator_phase_error(demod, &phi);

                // step pll, nco objects
                nco_crcf_pll_step(nco_rx, phi);
                nco_crcf_step(nco_rx);

                // append demodulated bits to byte
                byte <<= 2;
                byte |= demod_sym & 0x03;
                num_bits += 2;
                if (num_bits >=8) {
                    bpacketsync_execute(ps, byte);
                    byte = 0;
                    num_bits = 0;
                }
#if DEBUG
                if (file_open) {
                    fprintf(fid,"mfdecim_out(%5u) = %12.4e + j*%12.4e;\n", k+1, mfdecim_out[j].real(), mfdecim_out[j].imag());
                    fprintf(fid,"nco_out(%5u)     = %12.4e + j*%12.4e;\n", k+1, nco_out.real(), nco_out.imag());
                    fprintf(fid,"phi(%5u)         = %12.4e;\n", k+1, phi);
                    k++;
                }
#endif

            }
        }

#if 0
        // report agc rssi
        float rssi = agc_crcf_get_signal_level(agc_rx);
        printf("rssi = %12.8f\n", 10*log10f(rssi));
#endif

#if DEBUG
    if (file_open && (n == num_blocks-1 || n*rx_buffer_length > 4096)) {
        fprintf(fid,"figure; plot(mfdecim_out,'x'); axis square;\n");
        fprintf(fid,"figure; plot(nco_out,'x'); axis square;\n");
        fclose(fid);
        printf("results written to rx_rrc_debug.m\n");
        file_open = 0;
    }
#endif
    }
 
 
    uio->stop_rx(USRP_CHANNEL);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // print results
    float data_rate = num_valid_bytes_received * 8.0f / num_seconds;
    float percent_valid = (num_packets_received == 0) ?
                          0.0f :
                          100.0f * (float)num_valid_packets_received / (float)num_packets_received;
    printf("    packets received    : %6u\n", num_packets_received);
    printf("    valid packets       : %6u (%6.2f%%)\n", num_valid_packets_received,percent_valid);
    printf("    data rate           : %8.4f kbps\n", data_rate*1e-3f);

    // destroy synchronization objects
    agc_crcf_destroy(agc_rx);
    nco_crcf_destroy(nco_rx);
    symsync_crcf_destroy(mfdecim);
    modem_destroy(demod);
    bpacketsync_destroy(ps);

    delete uio;
    return 0;
}

