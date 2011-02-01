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
#include <liquid/liquid.h>

#include "usrp_io.h"
 
#define USRP_CHANNEL        (0)

void usage() {
    printf("gmsk_tx:\n");
    printf("  u,h   :   usage/help\n");
    printf("  f     :   center frequency [Hz], default: 462 MHz\n");
    printf("  s     :   symbol rate [Hz], default 62.5 kHz\n");
    printf("  g     :   transmit power gain [dB], default -3dB\n");
    printf("  B     :   bandwidth factor, default: 0.3\n");
    printf("  n     :   num packets [1024]\n");
}

int main (int argc, char **argv)
{
    float min_symbol_rate = (32e6 / 512.0);
    float max_symbol_rate = (32e6 /   4.0);

    float frequency = 462.0e6;
    float symbol_rate = min_symbol_rate;
    float txgain_dB = -3.0f;

    float BT = 0.3f;
    unsigned int num_packets = 1024;

    // packetizer properties
    unsigned int packet_len_dec = 64;   // original data message length
    crc_scheme check = CRC_32;          // data integrity check
    fec_scheme fec0 = FEC_HAMMING128;   // inner code
    fec_scheme fec1 = FEC_NONE;         // outer code

    //
    int d;
    while ((d = getopt(argc,argv,"uhf:s:g:B:n:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'f':   frequency = atof(optarg);       break;
        case 's':   symbol_rate = atof(optarg);     break;
        case 'g':   txgain_dB = atof(optarg);       break;
        case 'B':   BT = atof(optarg);              break;
        case 'n':   num_packets = atof(optarg);     break;
        default:
            fprintf(stderr,"error: unknown/invalid option\n");
            return 0;
        }
    }

    if (symbol_rate > max_symbol_rate) {
        fprintf(stderr,"error: maximum symbol rate exceeded (%8.4f MHz)\n", max_symbol_rate*1e-6);
        return 0;
    } else if (symbol_rate < min_symbol_rate) {
        fprintf(stderr,"error: minimum symbol rate exceeded (%8.4f kHz)\n", min_symbol_rate*1e-3);
        return 0;
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("symbol rate :   %12.8f [kHz]\n", symbol_rate*1e-3f);
    printf("tx gain     :   %12.8f [dB]\n", txgain_dB);

    unsigned int k=2;   // filter samples/symbol
    unsigned int m=3;   // filter delay

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_tx_freq(USRP_CHANNEL, frequency);
    uio->set_tx_samplerate(k*symbol_rate);
    uio->enable_auto_tx(USRP_CHANNEL);

    // retrieve tx port
    gport port_tx = uio->get_tx_port(USRP_CHANNEL);

    // create packet generator
    bpacketgen pg = bpacketgen_create(0, packet_len_dec, check, fec0, fec1);
    bpacketgen_print(pg);
    unsigned int packet_len_enc = bpacketgen_get_packet_len(pg);

    // data arrays
    unsigned char packet_dec[packet_len_dec];
    unsigned char packet_enc[packet_len_enc];
    std::complex<float> buffer[packet_len_enc*8*k];
    
    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/10.0f);

    // create GMSK modem
    gmskmod mod = gmskmod_create(k,m,BT);

 
    unsigned int i;
    unsigned int j;
    unsigned int n;
    // start transmitter
    uio->start_tx(USRP_CHANNEL);
    for (n=0; n<num_packets; n++) {

        // generate random payload
        for (i=0; i<packet_len_dec; i++)
            packet_dec[i] = rand() & 0xff;

        // encode packet
        bpacketgen_encode(pg, packet_dec, packet_enc);

        // modulate data symbols
        for (i=0; i<packet_len_enc; i++) {
            unsigned int j;
            for (j=0; j<8; j++) {
                unsigned int sym = (packet_enc[i] >> (8-j-1)) & 1;
                gmskmod_modulate(mod, sym, &buffer[(8*i+j)*k]);
            }
        }
        
#if 0
        // apply gain
        for (j=0; j<k*frame_len; j++)
            buffer[j] *= g;
#endif

        // send data to usrp via port
        gport_produce(port_tx,(void*)buffer,8*k*packet_len_enc);
 
    }
 
    uio->stop_tx(USRP_CHANNEL);  // Stop data transfer

    // delete objects
    gmskmod_destroy(mod);
    bpacketgen_destroy(pg);
    delete uio;

    return 0;
}

