/*
 * Copyright (c) 2011, 2013 Joseph Gaeddert
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

#include "ofdmtxrx.h"

void usage() {
    printf("ofdmflexframe_tx [OPTION]\n");
    printf("transmit OFDM packets\n");
    printf("\n");
    printf("  u,h   : usage/help\n");
    printf("  q/v   : quiet/verbose\n");
    printf("  f     : center frequency [Hz],  default:  462 MHz\n");
    printf("  b     : bandwidth [Hz],         default: 1000 kHz\n");
    printf("  g     : software tx gain [dB],  default:  -12 dB \n");
    printf("  G     : uhd tx gain [dB],       default:   40 dB\n");
    printf("  N     : number of frames,       default: 2000\n");
    printf("  M     : number of subcarriers,  default:   48\n");
    printf("  C     : cyclic prefix length,   default:    6\n");
    printf("  T     : taper length,           default:    4\n");
    printf("  P     : payload length [bytes], default: 1200 bytes\n");
    printf("  m     : modulation scheme,      default: qpsk\n");
    liquid_print_modulation_schemes();
    printf("  c     : coding scheme (inner),  default: g2412\n");
    printf("  k     : coding scheme (outer),  default: none\n");
    liquid_print_fec_schemes();
}

int main (int argc, char **argv)
{
    // command-line options
    bool verbose = true;

    double frequency = 462.0e6;         // carrier frequency
    double bandwidth = 1000e3f;         // bandwidth
    unsigned int num_frames = 2000;     // number of frames to transmit
    double txgain_dB = -12.0f;          // software tx gain [dB]
    double uhd_txgain = 40.0;           // uhd (hardware) tx gain

    // ofdm properties
    unsigned int M = 48;                // number of subcarriers
    unsigned int cp_len = 6;            // cyclic prefix length
    unsigned int taper_len = 4;         // taper length

    modulation_scheme ms = LIQUID_MODEM_QPSK;// modulation scheme
    unsigned int payload_len = 1200;        // original data message length
    //crc_scheme check = LIQUID_CRC_32;       // data validity check
    fec_scheme fec0 = LIQUID_FEC_NONE;      // fec (inner)
    fec_scheme fec1 = LIQUID_FEC_GOLAY2412; // fec (outer)
    
    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:g:G:N:M:C:T:P:m:c:k:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose     = false;            break;
        case 'v':   verbose     = true;             break;
        case 'f':   frequency   = atof(optarg);     break;
        case 'b':   bandwidth   = atof(optarg);     break;
        case 'g':   txgain_dB   = atof(optarg);     break;
        case 'G':   uhd_txgain  = atof(optarg);     break;
        case 'N':   num_frames  = atoi(optarg);     break;
        case 'M':   M           = atoi(optarg);     break;
        case 'C':   cp_len      = atoi(optarg);     break;
        case 'T':   taper_len   = atoi(optarg);     break;
        case 'P':   payload_len = atoi(optarg);     break;
        case 'm':   ms          = liquid_getopt_str2mod(optarg);    break;
        case 'c':   fec0        = liquid_getopt_str2fec(optarg);    break;
        case 'k':   fec1        = liquid_getopt_str2fec(optarg);    break;
        default:    usage();                        return 0;
        }
    }

    if (cp_len == 0 || cp_len > M) {
        fprintf(stderr,"error: %s, cyclic prefix must be in (0,M]\n", argv[0]);
        exit(1);
    } else if (ms == LIQUID_MODEM_UNKNOWN) {
        fprintf(stderr,"error: %s, unknown/unsupported mod. scheme\n", argv[0]);
        exit(-1);
    } else if (fec0 == LIQUID_FEC_UNKNOWN) {
        fprintf(stderr,"error: %s, unknown/unsupported inner fec scheme\n", argv[0]);
        exit(-1);
    } else if (fec1 == LIQUID_FEC_UNKNOWN) {
        fprintf(stderr,"error: %s, unknown/unsupported outer fec scheme\n", argv[0]);
        exit(-1);
    }

    // create transceiver object
    ofdmtxrx txcvr(M, cp_len, taper_len, NULL, NULL);

    // set properties
    txcvr.set_tx_freq(frequency);
    txcvr.set_tx_rate(bandwidth);
    txcvr.set_tx_gain_soft(txgain_dB);
    txcvr.set_tx_gain_uhd(uhd_txgain);

    // data arrays
    unsigned char header[8];
    unsigned char payload[payload_len];
    
    unsigned int pid;
    unsigned int i;
    for (pid=0; pid<num_frames; pid++) {
        if (verbose)
            printf("tx packet id: %6u\n", pid);
        
        // write header (first two bytes packet ID, remaining are random)
        header[0] = (pid >> 8) & 0xff;
        header[1] = (pid     ) & 0xff;
        for (i=2; i<8; i++)
            header[i] = rand() & 0xff;

        // initialize payload
        for (i=0; i<payload_len; i++)
            payload[i] = rand() & 0xff;

        // transmit frame
        txcvr.transmit_packet(header, payload, payload_len, ms, fec0, fec1);

    } // packet loop
 
    // sleep for a small amount of time to allow USRP buffers
    // to flush
    usleep(200000);

    //finished
    printf("usrp data transfer complete\n");

    printf("done.\n");
    return 0;
}

