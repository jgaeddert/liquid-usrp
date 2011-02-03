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
    printf("packetstream_tx -- transmit streaming DPSK packets\n");
    printf("  u,h   :   usage/help\n");
    printf("  q/v   :   quiet/verbose\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  s     :   symbol rate [Hz] (62.5kHz min, 8MHz max)\n");
    printf("  N     :   number of packets [1024]\n");
    printf("  n     :   payload length (bytes)\n");
    printf("  m     :   filter delay [symbols]\n");
    printf("  b     :   filter excess bandwidth factor [0.0 min, 1.0 max]\n");
    printf("  p     :   modulation depth [bits/symbol], default: 2\n");
    printf("  c     :   fec coding scheme (inner)\n");
    printf("  k     :   fec coding scheme (outer)\n");
    // print all available FEC schemes
    unsigned int i;
    for (i=0; i<LIQUID_NUM_FEC_SCHEMES; i++)
        printf("          [%s] %s\n", fec_scheme_str[i][0], fec_scheme_str[i][1]);
}

int main (int argc, char **argv)
{
    // command-line options
    bool verbose = true;

    float min_symbol_rate = (32e6 / 512.0);
    float max_symbol_rate = (32e6 /   4.0);

    float frequency = 462.0e6;
    float symbol_rate = min_symbol_rate;
    //float num_seconds = 5.0f;
    unsigned int num_packets = 1024;
    unsigned int m = 3;
    float beta = 0.3f;

    // packetizer properties
    unsigned int packet_len_dec = 64;   // original data message length
    crc_scheme check = CRC_32;          // data integrity check
    fec_scheme fec0 = FEC_HAMMING128;   // inner code
    fec_scheme fec1 = FEC_NONE;         // outer code
    modulation_scheme ms = MOD_DPSK;    // modulation scheme (fixed)
    unsigned int bps = 2;               // modulation depth

    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:s:N:n:m:b:p:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose = false;                break;
        case 'v':   verbose = true;                 break;
        case 'f':   frequency = atof(optarg);       break;
        case 's':   symbol_rate = atof(optarg);     break;
        //case 't':   num_seconds = atof(optarg);     break;
        case 'N':   num_packets = atoi(optarg);     break;
        case 'n':   packet_len_dec = atoi(optarg);  break;
        case 'm':   m = atoi(optarg);               break;
        case 'b':   beta = atof(optarg);            break;
        case 'p':   bps = atoi(optarg);             break;
        case 'c':
            fec0 = liquid_getopt_str2fec(optarg);
            if (fec0 == FEC_UNKNOWN) {
                fprintf(stderr,"error: %s, unknown/unsupported fec scheme '%s'\n", argv[0], optarg);
                exit(1);
            }
            break;
        case 'k':
            fec1 = liquid_getopt_str2fec(optarg);
            if (fec1 == FEC_UNKNOWN) {
                fprintf(stderr,"error: %s, unknown/unsupported fec scheme '%s'\n", argv[0], optarg);
                exit(1);
            }
            break;
        default:
            usage();
            return 0;
        }
    }

    if (symbol_rate > max_symbol_rate) {
        fprintf(stderr,"error: %s, maximum symbol_rate exceeded (%8.4f MHz)\n", argv[0], max_symbol_rate*1e-6);
        return 0;
    } else if (symbol_rate < min_symbol_rate) {
        fprintf(stderr,"error: %s, minimum symbol_rate exceeded (%8.4f kHz)\n", argv[0], min_symbol_rate*1e-3);
        exit(1);
    } else if (m < 1 || m > 20) {
        fprintf(stderr,"error: %s, filter length m must be in [1,20]\n", argv[0]);
        exit(1);
    } else if (beta < 0.0f || beta > 1.0f) {
        fprintf(stderr,"error: %s, filter excess bandwidth beta must be in [0,1]\n", argv[0]);
        exit(1);
    } else if (bps == 0 || bps > 8) {
        fprintf(stderr,"error: %s, modulation depth must be in [1,8]\n", argv[0]);
        exit(1);
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("symbol_rate :   %12.8f [kHz]\n", symbol_rate*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    // 
    //unsigned int num_blocks = (unsigned int)((4.0f*symbol_rate*num_seconds)/(512));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_tx_freq(0, frequency);
    uio->set_tx_samplerate(2.0f*symbol_rate);
    uio->enable_auto_tx(0);

    // retrieve tx port from usrp_io object
    gport port_tx = uio->get_tx_port(0);

    // filter parameters
    unsigned int k=2;   // samples/symbol
    //unsigned int m=3;   // symbol delay
    float dt=0.0f;      // fractional sample delay
    //float beta=0.3f;    // excess bandwidth factor

    // create matched-filter interpolator
    interp_crcf nyquist_filter = interp_crcf_create_rrc(k,m,beta,dt);

    // create modem
    modem mod = modem_create(ms,bps);

    unsigned int n; // sample counter

    // create packet generator
    bpacketgen pg = bpacketgen_create(0, packet_len_dec, check, fec0, fec1);
    bpacketgen_print(pg);
    unsigned int packet_len_enc = bpacketgen_get_packet_len(pg);

    // data arrays
    unsigned char packet_dec[packet_len_dec];
    unsigned char packet_enc[packet_len_enc];
    
    // 
    div_t dsyms = div(8*packet_len_enc,bps);
    unsigned int num_symbols = dsyms.quot + (dsyms.rem ? 1 : 0);
    std::complex<float> symbols[num_symbols];
    std::complex<float> data_tx[2*num_symbols];

    unsigned int i;

    // start USRP data transfer
    uio->start_tx(0);
    for (i=0; i<num_packets; i++) {

        // generate random payload
        for (n=0; n<packet_len_dec; n++)
            packet_dec[n] = rand() & 0xff;

        // encode packet
        bpacketgen_encode(pg, packet_dec, packet_enc);

        // generate data symbols (unpack array)
        for (n=0; n<num_symbols; n++) {
            unsigned char sym=0;
            liquid_unpack_array(packet_enc,
                                packet_len_enc,
                                n*bps,  // bit index
                                bps,    // symbol size
                                &sym);  // output symbol
            modem_modulate(mod, sym, &symbols[n]);
        }

        // run nyquist filter/interpolator
        for (n=0; n<num_symbols; n++)
            interp_crcf_execute(nyquist_filter,symbols[n],&data_tx[2*n]);

        gport_produce(port_tx,(void*)data_tx,2*num_symbols);
    }
 
    uio->stop_tx(0);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // clean it up
    bpacketgen_destroy(pg);
    interp_crcf_destroy(nyquist_filter);
    modem_destroy(mod);
    delete uio;
    return 0;
}

