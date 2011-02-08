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
    printf("ofdmframe_tx -- transmit OFDM packets\n");
    printf("  u,h   :   usage/help\n");
    printf("  q/v   :   quiet/verbose\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  b     :   bandwidth [Hz] (62.5kHz min, 8MHz max)\n");
    printf("  M     :   number of subcarriers, default: 64\n");
    printf("  C     :   cyclic prefix length, default: 16\n");
    printf("  m     :   modulation scheme: psk, dpsk, ask, <qam>, apsk\n");
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

    float min_bandwidth = (32e6 / 512.0);
    float max_bandwidth = (32e6 /   4.0);

    float frequency = 462.0e6;
    float bandwidth = min_bandwidth;
    //float num_seconds = 5.0f;

    unsigned int M = 64;                // number of subcarriers
    unsigned int cp_len = 16;           // cyclic prefix length
    unsigned int num_symbols_S0 = 2;    // number of S0 symbols
    unsigned int num_symbols_S1 = 2;    // number of S0 symbols

    modulation_scheme ms = MOD_QAM;     // modulation scheme
    unsigned int bps = 2;               // modulation depth
    unsigned int packet_len_dec = 50;   // original data message length
    crc_scheme check = CRC_32;          // data validity check
    fec_scheme fec0 = FEC_NONE;         // fec (inner)
    fec_scheme fec1 = FEC_HAMMING128;   // fec (outer)

    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:M:C:m:p:c:k:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose = false;                break;
        case 'v':   verbose = true;                 break;
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 'M':   M = atoi(optarg);               break;
        case 'C':   cp_len = atoi(optarg);          break;
        case 'm':
            ms = liquid_getopt_str2mod(optarg);
            if (ms == MOD_UNKNOWN) {
                fprintf(stderr, "error: %s unknown/unsupported mod. scheme: %s\n", argv[0], optarg);
                ms = MOD_UNKNOWN;
            }
            break;
        case 'p':   bps = atoi(optarg);             break;
        case 'c':   fec0 = liquid_getopt_str2fec(optarg);   break;
        case 'k':   fec1 = liquid_getopt_str2fec(optarg);   break;
        default:
            usage();
            return 0;
        }
    }

    unsigned int i;

    if (bandwidth > max_bandwidth) {
        fprintf(stderr,"error: %s, maximum bandwidth exceeded (%8.4f MHz)\n", argv[0], max_bandwidth*1e-6);
        return 0;
    } else if (bandwidth < min_bandwidth) {
        fprintf(stderr,"error: %s, minimum bandwidth exceeded (%8.4f kHz)\n", argv[0], min_bandwidth*1e-3);
        exit(1);
    } else if (cp_len == 0 || cp_len > M) {
        fprintf(stderr,"error: %s, cyclic prefix must be in (0,M]\n", argv[0]);
        exit(1);
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    // 
    //unsigned int num_blocks = (unsigned int)((4.0f*bandwidth*num_seconds)/(512));


    // initialize subcarrier allocation
    unsigned int p[M];
    unsigned int guard = M / 6;
    unsigned int pilot_spacing = 8;
    unsigned int i0 = (M/2) - guard;
    unsigned int i1 = (M/2) + guard;
    for (i=0; i<M; i++) {
        if ( i == 0 || (i > i0 && i < i1) )
            p[i] = OFDMFRAME_SCTYPE_NULL;
        else if ( (i%pilot_spacing)==0 )
            p[i] = OFDMFRAME_SCTYPE_PILOT;
        else
            p[i] = OFDMFRAME_SCTYPE_DATA;
    }

    unsigned int M_null=0;
    unsigned int M_pilot=0;
    unsigned int M_data=0;
    ofdmframe_validate_sctype(p,M, &M_null, &M_pilot, &M_data);

    // number of ofdm data symbols in frame (excluding preamble)
    unsigned int num_symbols_data = (2800/M_data) + 1;

    // total number of modulated symbols in frame
    unsigned int num_mod_symbols = num_symbols_data * M_data;
    
    // create packet generator
    bpacketgen pg = bpacketgen_create(0, packet_len_dec, check, fec0, fec1);
    bpacketgen_print(pg);
    unsigned int packet_len_enc = bpacketgen_get_packet_len(pg);

    // data arrays
    unsigned char packet_dec[packet_len_dec];
    unsigned char packet_enc[packet_len_enc];
    
    // 
    div_t dsyms = div(8*packet_len_enc,bps);
    unsigned int num_packet_symbols = dsyms.quot + (dsyms.rem ? 1 : 0);
    unsigned int packets_per_frame = num_mod_symbols / num_packet_symbols;

    printf("ofdmframe_tx properties\n");
    printf("    null            :   %u\n", M_null);
    printf("    plot            :   %u\n", M_pilot);
    printf("    data            :   %u\n", M_data);
    printf("    ofdm syms/frame :   %u\n", num_symbols_data);
    printf("    mod. syms/frame :   %u\n", num_mod_symbols);
    printf("    packet len dec  :   %u bytes\n", packet_len_dec);
    printf("    packet len enc  :   %u bytes\n", packet_len_enc);
    printf("    mod. syms/packet:   %u\n", num_packet_symbols);
    printf("    packets/frame   :   %u\n", packets_per_frame);

    // create frame generator
    ofdmframegen fg = ofdmframegen_create(M, cp_len, p);
    ofdmframegen_print(fg);

    // arrays
    std::complex<float> modsyms[num_packet_symbols];
    std::complex<float> X[M];           // channelized symbols
    std::complex<float> S0[M];          // PLCP sequence (short)
    std::complex<float> S1[M];          // PLCP sequence (long)
    std::complex<float> y[M+cp_len];    // output time series

    // initialize sequence arrays
    ofdmframegen_write_S0(fg, S0);
    ofdmframegen_write_S1(fg, S1);

    // create modem
    modem mod = modem_create(ms,bps);

    unsigned int j;
    unsigned int n;

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_tx_freq(0, frequency);
    uio->set_tx_samplerate(2.0f*bandwidth);
    uio->set_tx_gain(0, 0.1f);
    uio->enable_auto_tx(0);

    // retrieve tx port from usrp_io object
    gport port_tx = uio->get_tx_port(0);

    // start USRP data transfer
    uio->start_tx(0);
    unsigned int num_frames = -1;

    for (i=0; i<num_frames; i++) {

        //
        // preamble
        //

        // write short sequence(s)
        for (n=0; n<num_symbols_S0; n++)
            gport_produce(port_tx, (void*)S0, M);

        // write long sequence extension
        gport_produce(port_tx, (void*)( &S1[M-cp_len] ), cp_len);

        // write long sequence(s)
        for (n=0; n<num_symbols_S1; n++)
            gport_produce(port_tx, (void*)S1, M);

        //
        // payload
        //
        
        // payload modem symbol counter
        unsigned int k=0;           // modem symbol counter
        unsigned int num_packets=0; // packet counter

        // generate ofdm payload symbols
        for (n=0; n<num_symbols_data; n++) {

            // load data onto subcarriers
            for (j=0; j<M; j++) {

                // ignore non-data subcarriers
                if (p[j] != OFDMFRAME_SCTYPE_DATA)
                    continue;

                // generate payload
                if (k==0 && num_packets < packets_per_frame) {
                    unsigned int ii;
                    for (ii=0; ii<packet_len_dec; ii++)
                        packet_dec[ii] = rand() & 0xff;

                    // encode packet
                    bpacketgen_encode(pg, packet_dec, packet_enc);

                    // generate data symbols
                    for (ii=0; ii<num_packet_symbols; ii++) {
                        unsigned char sym=0;
                        liquid_unpack_array(packet_enc,
                                            packet_len_enc,
                                            ii*bps,     // bit index
                                            bps,        // symbol size
                                            &sym);      // output symbol

                        // modulate symbol
                        modem_modulate(mod, sym, &modsyms[ii]);
                    }
                    num_packets++;
                } else if (k==0 && num_packets >= packets_per_frame) {
                    //printf("random payload\n");
                    unsigned int ii;
                    for (ii=0; ii<num_packet_symbols; ii++) {
                        unsigned char sym = modem_gen_rand_sym(mod);
                        modem_modulate(mod, sym, &modsyms[ii]);
                    }
                }

                // store symbol to ofdm input subcarrier array
                X[j] = modsyms[k];

                // update counter
                k = (k+1) % num_packet_symbols;
            }

            // generate ofdm payload symbol
            ofdmframegen_writesymbol(fg, X, y);

            gport_produce(port_tx, (void*)y, M+cp_len);
        }

        // flush with zeros
        //for (n=0; n<M+cp_len; n++)

        // reset frame generator (resets pilot generator, etc.)
        ofdmframegen_reset(fg);

        if (verbose)
            printf("frame transmitted %2u / %2u packets\n", num_packets, packets_per_frame);
    }
 
    uio->stop_tx(0);  // Stop data transfer
    printf("usrp data transfer complete\n");

    // clean it up
    ofdmframegen_destroy(fg);
    modem_destroy(mod);
    bpacketgen_destroy(pg);
    delete uio;
    return 0;
}

