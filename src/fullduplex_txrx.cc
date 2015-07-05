/*
 * Copyright (c) 2015 Joseph Gaeddert
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
#include <pthread.h>
#include <liquid/liquid.h>

#include "ofdmtxrx.h"
#include "timer.h"

void usage() {
    printf("fullduplex_txrx [OPTION]\n");
    printf("transmit OFDM packets back and forth\n");
    printf("\n");
    printf("  h     : usage/help\n");
    printf("  v/q   : verbose/quiet\n");
    printf("  f     : tx frequency [Hz],        default:  462 MHz\n");
    printf("  o     : rx frequency offset [Hz], default:  +40 MHz\n");
    printf("  R     : reverse tx/rx frequencies,default: false\n");
    printf("  b     : bandwidth [Hz],           default:  400 kHz\n");
    printf("  g     : software tx gain [dB],    default:  -12 dB\n");
    printf("  G     : uhd tx gain [dB],         default:   40 dB\n");
    printf("  N     : number of frames,         default: 1000\n");
    printf("  M     : number of subcarriers,    default:   64\n");
    printf("  C     : cyclic prefix length,     default:    8\n");
    printf("  T     : taper length,             default:    4\n");
    printf("  P     : payload length,           default:  800 bytes\n");
    printf("  m     : modulation scheme,        defulat: qpsk\n");
    liquid_print_modulation_schemes();
    printf("  c     : coding scheme (inner),    default: h128\n");
    printf("  k     : coding scheme (outer),    default: none\n");
    liquid_print_fec_schemes();
    printf("  d     :   enable debugging mode\n");
}

// receiver callback function
int callback(unsigned char *  _header,
             int              _header_valid,
             unsigned char *  _payload,
             unsigned int     _payload_len,
             int              _payload_valid,
             framesyncstats_s _stats,
             void *           _userdata);

int main (int argc, char **argv)
{
    // command-line options
    bool verbose = true;

    double       offset     =  100e6f;  // rx frequency offset
    int          reverse_txrx   = 0;        // reverse tx/rx frequencies
    double       frequency  = 462.0e6;  // carrier frequency
    double       bandwidth  = 1000e3f;  // bandwidth
    unsigned int num_frames = 2000;     // number of frames to transmit
    double       txgain_dB  = -12.0f;   // software tx gain [dB]
    double       uhd_txgain = 40.0;     // uhd (hardware) tx gain
    float        uhd_rxgain = 20.0;

    // ofdm properties
    unsigned int M = 48;                // number of subcarriers
    unsigned int cp_len = 6;            // cyclic prefix length
    unsigned int taper_len = 4;         // taper length

    modulation_scheme ms = LIQUID_MODEM_QPSK;// modulation scheme
    unsigned int payload_len = 1200;        // original data message length
    //crc_scheme check = LIQUID_CRC_32;       // data validity check
    fec_scheme fec0 = LIQUID_FEC_NONE;      // fec (inner)
    fec_scheme fec1 = LIQUID_FEC_GOLAY2412; // fec (outer)
    int debug_enabled =  0;             // enable debugging?
    
    //
    int d;
    while ((d = getopt(argc,argv,"hvqf:o:Rb:g:G:N:M:C:T:P:m:c:k:d")) != EOF) {
        switch (d) {
        case 'h':   usage();                        return 0;
        case 'v':   verbose     = true;             break;
        case 'q':   verbose     = false;            break;
        case 'f':   frequency   = atof(optarg);     break;
        case 'o':   offset      = atof(optarg);     break;
        case 'R':   reverse_txrx= 1;                break;
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
        case 'd':   debug_enabled = 1;                  break;
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
    unsigned char * p = NULL;   // default subcarrier allocation
    ofdmtxrx txcvr(M, cp_len, taper_len, p, callback, (void*)&bandwidth);

    // set properties
    txcvr.set_tx_freq(frequency + (reverse_txrx ? offset : 0.0f));
    txcvr.set_tx_rate(bandwidth);
    txcvr.set_tx_gain_soft(txgain_dB);
    txcvr.set_tx_gain_uhd(uhd_txgain);

    // set properties
    txcvr.set_rx_freq(frequency + (reverse_txrx ? 0.0f : offset));
    txcvr.set_rx_rate(bandwidth);
    txcvr.set_rx_gain_uhd(uhd_rxgain);

    // enable debugging on request
    if (debug_enabled)
        txcvr.debug_enable();

    // start receiver
    txcvr.start_rx();

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

    // stop receiver
    printf("ofdmflexframe_rx stopping receiver...\n");
    txcvr.stop_rx();
 
    //finished
    printf("usrp data transfer complete\n");

    printf("done.\n");
    return 0;
}

// callback function
int callback(unsigned char *  _header,
             int              _header_valid,
             unsigned char *  _payload,
             unsigned int     _payload_len,
             int              _payload_valid,
             framesyncstats_s _stats,
             void *           _userdata)
{
    // compute true carrier offset
    float samplerate = *((float*)_userdata);
    float cfo = _stats.cfo * samplerate / (2*M_PI);
    printf("***** rssi=%7.2fdB evm=%7.2fdB, cfo=%7.3f kHz, ", _stats.rssi, _stats.evm, cfo*1e-3f);

    if (_header_valid) {
        unsigned int packet_id = (_header[0] << 8 | _header[1]);
        printf("rx packet id: %6u", packet_id);
        if (_payload_valid) printf("\n");
        else                printf(" PAYLOAD INVALID\n");
    } else {
        printf("HEADER INVALID\n");
    }

#if 0
    // update global counters
    num_frames_detected++;

    if (_header_valid)
        num_valid_headers_received++;

    if (_payload_valid) {
        num_valid_packets_received++;
        num_valid_bytes_received += _payload_len;
    }
#endif
    return 0;
}

