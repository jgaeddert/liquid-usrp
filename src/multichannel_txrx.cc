/*
 * Copyright (c) 2013 Joseph Gaeddert
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

// 
// multichannel_txrx.cc
//
// This program demonstrates the functionality of the multichannel
// transceiver class (see include/multichanneltxrx.h) which allows
// concurrent transmission of data on multiple OFDM channels, each
// with a potentially different modulation/coding scheme, payload
// length, etc.
//
// After initialization, the basic program runs as follows:
//  1. turn on the transmitter
//  2. transmit a burst of packets on all channels, each with a
//     random payload length
//  3. after a certain time, stop generating new packets, but wait
//     for the device to finish sending packets already scheduled
//  4. turn off the transmitter
//  5. turn on the receiver
//  6. wait for a specified amount of time while receiving packets
//     (received packets will be piped through the user-defined
//     callback function)
//  7. turn off the receiver
//  8. repeat this entire sequence until a certain amount of total
//     time has elapsed.
//
 
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <complex>
#include <getopt.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <liquid/liquid.h>
#include <assert.h>

#include "multichanneltxrx.h"
#include "timer.h"

void usage() {
    printf("multichannel_txrx [OPTION]\n");
    printf("transmit OFDM packets back and forth over multiple channels simultaneously\n");
    printf("\n");
    printf("  u,h   : usage/help\n");
    printf("  q/v   : quiet/verbose\n");
    printf("  f     : center frequency [Hz],  default:  462 MHz\n");
    printf("  b     : bandwidth [Hz],         default: 1000 kHz\n");
    printf("  g     : software tx gain [dB],  default:  -12 dB \n");
    printf("  G     : uhd tx gain [dB],       default:   40 dB\n");
    printf("  M     : number of subcarriers,  default:   48\n");
    printf("  C     : cyclic prefix length,   default:    6\n");
    printf("  T     : taper length,           default:    4\n");
    printf("  n     : number of channels,     default: 2\n");
    printf("  P     : payload length [bytes], default: 1200 bytes\n");
    printf("  m     : modulation scheme,      default: qpsk\n");
    liquid_print_modulation_schemes();
    printf("  c     : coding scheme (inner),  default: g2412\n");
    printf("  k     : coding scheme (outer),  default: none\n");
    liquid_print_fec_schemes();
    printf("  t     : total runtime [s],      default:   30 s\n");
}

// assemble packet
void assemble_packet(unsigned int    _pid,
                     unsigned char * _header,
                     unsigned char * _payload,
                     unsigned int    _payload_len);

// set timespec for timeout
//  _ts         :   pointer to timespec structure
//  _timeout    :   time before timeout
void set_timespec(struct timespec * _ts,
                  float             _timeout);

// callback function
int callback(unsigned char *  _header,
             int              _header_valid,
             unsigned char *  _payload,
             unsigned int     _payload_len,
             int              _payload_valid,
             framesyncstats_s _stats,
             void *           _userdata);

static bool verbose = true;

// data counters
unsigned int num_frames_detected;
unsigned int num_valid_headers_received;
unsigned int num_valid_packets_received;
unsigned int num_valid_bytes_received;

int main (int argc, char **argv)
{
    // command-line options
    float frequency = 462.0e6;          // carrier frequency
    float bandwidth = 1000e3f;          // bandwidth
    float txgain_dB = -12.0f;           // software tx gain [dB]
    float uhd_txgain = 40.0;            // uhd (hardware) tx gain
    float uhd_rxgain = 20.0;            // uhd (hardware) rx gain
    
    unsigned int num_channels = 2;      // number of OFDM channels

    // ofdm properties
    unsigned int M = 48;                // number of subcarriers
    unsigned int cp_len = 6;            // cyclic prefix length
    unsigned int taper_len = 4;         // taper length

    modulation_scheme ms = LIQUID_MODEM_QPSK;// modulation scheme
    unsigned int payload_len = 1200;        // original data message length
    //crc_scheme check = LIQUID_CRC_32;       // data validity check
    fec_scheme fec0 = LIQUID_FEC_NONE;      // fec (inner)
    fec_scheme fec1 = LIQUID_FEC_GOLAY2412; // fec (outer)

    // timers
    float tx_burst_time = 0.250;        // time of transmit burst
    float rx_burst_time = 2.500;        // time of receive burst
    float runtime       = 30.00;        // total run time
    
    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:g:G:M:C:T:n:P:m:c:k:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose     = false;            break;
        case 'v':   verbose     = true;             break;
        case 'f':   frequency   = atof(optarg);     break;
        case 'b':   bandwidth   = atof(optarg);     break;
        case 'g':   txgain_dB   = atof(optarg);     break;
        case 'G':   uhd_txgain  = atof(optarg);     break;
        case 'M':   M           = atoi(optarg);     break;
        case 'C':   cp_len      = atoi(optarg);     break;
        case 'T':   taper_len   = atoi(optarg);     break;
        case 'n':   num_channels= atoi(optarg);     break;
        case 'P':   payload_len = atoi(optarg);     break;
        case 'm':   ms          = liquid_getopt_str2mod(optarg);    break;
        case 'c':   fec0        = liquid_getopt_str2fec(optarg);    break;
        case 'k':   fec1        = liquid_getopt_str2fec(optarg);    break;
        case 't':   runtime     = atof(optarg);     break;
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
    } else if (num_channels == 0) {
        fprintf(stderr,"error: %s, number of channels must be greater than zero\n", argv[0]);
        exit(-1);
    }

    unsigned int i;

    // thread handling
    pthread_mutex_t rx_mutex;   // receiver mutex
    pthread_cond_t  rx_cond;    // receiver condition
    pthread_mutex_init(&rx_mutex, NULL);
    pthread_cond_init(&rx_cond,   NULL);

    // create transceiver object
    void * userdata[num_channels];
    framesync_callback callbacks[num_channels];
    for (i=0; i<num_channels; i++) {
        userdata[i] = NULL;  //(void*)&rx_cond);
        callbacks[i] = callback;
    }
    unsigned char * p = NULL;   // default subcarrier allocation
    multichanneltxrx txcvr(num_channels, M, cp_len, taper_len, p, callbacks, userdata);

    // set transmit properties
    txcvr.set_tx_freq(frequency);
    txcvr.set_tx_rate(bandwidth);
    txcvr.set_tx_gain_soft(txgain_dB);
    txcvr.set_tx_gain_uhd(uhd_txgain);

    // set receive properties
    txcvr.set_rx_freq(frequency);
    txcvr.set_rx_rate(bandwidth);
    txcvr.set_rx_gain_uhd(uhd_rxgain);

    // data arrays
    unsigned char header[8];
    unsigned char payload[payload_len];

    // packet counter
    unsigned int pid=0;
    
    // reset counters
    num_frames_detected=0;
    num_valid_headers_received=0;
    num_valid_packets_received=0;
    num_valid_bytes_received=0;
    
    // create timers
    timer timer_runtime = timer_create();    timer_tic(timer_runtime);
    timer timer_tx      = timer_create();    timer_tic(timer_tx);
    //timer timer_rx      = timer_create();    timer_tic(timer_rx);
    while ( timer_toc(timer_runtime) < runtime ) {
        //if (verbose) printf("tx packet id: %6u\n", pid);

        // reset tx burst timer
        timer_tic(timer_tx);

        // transit a burst of packets
        txcvr.start_tx();
        printf("transmitting burst...\n");
        while ( timer_toc(timer_tx) < tx_burst_time) {
            // get next available channel (blocking)
            unsigned int c = txcvr.get_available_channel();
            assert( c < num_channels);

            // assemble packet
            unsigned int this_packet_len = rand() % payload_len;
            assemble_packet(pid, header, payload, this_packet_len);
            
            // transmit frame on channel 'c'
            printf("transmitting packet %6u (%6u bytes) on channel %6u\n", pid, this_packet_len, c);
            //int rc =
            txcvr.transmit_packet(c, header, payload, this_packet_len, ms, fec0, fec1);

            // update packet counter on channel 'c'
            pid++;
        }

        // wait for transmission to finish
        txcvr.wait_for_tx_to_complete();
        txcvr.stop_tx();

        // start receiver
        txcvr.start_rx();

        // sleep for prescribed time
        usleep(rx_burst_time*1e6f);

        // stop receiver
        txcvr.stop_rx();

    } // runtime loop
 
    // sleep for a small amount of time to allow USRP buffers
    // to flush
    usleep(200000);

    //finished
    printf("usrp data transfer complete\n");

    // compute actual run-time
    runtime = timer_toc(timer_runtime);

    // print results
    float data_rate = num_valid_bytes_received * 8.0f / runtime;
    float percent_headers_valid = (num_frames_detected == 0) ?
                          0.0f :
                          100.0f * (float)num_valid_headers_received / (float)num_frames_detected;
    float percent_packets_valid = (num_frames_detected == 0) ?
                          0.0f :
                          100.0f * (float)num_valid_packets_received / (float)num_frames_detected;
    printf("    frames detected     : %6u\n", num_frames_detected);
    printf("    valid headers       : %6u (%6.2f%%)\n", num_valid_headers_received,percent_headers_valid);
    printf("    valid packets       : %6u (%6.2f%%)\n", num_valid_packets_received,percent_packets_valid);
    printf("    bytes received      : %6u\n", num_valid_bytes_received);
    printf("    run time            : %f s\n", runtime);
    printf("    data rate           : %8.4f kbps\n", data_rate*1e-3f);

    // destroy objects
    timer_destroy(timer_runtime);
    timer_destroy(timer_tx);
    //timer_destroy(timer_rx);
    pthread_mutex_destroy(&rx_mutex);
    pthread_cond_destroy(&rx_cond);

    printf("done.\n");
    return 0;
}

// assemble packet
void assemble_packet(unsigned int    _pid,
                     unsigned char * _header,
                     unsigned char * _payload,
                     unsigned int    _payload_len)
{
    // write header (first two bytes packet ID, remaining are random)
    _header[0] = (_pid >> 8) & 0xff;
    _header[1] = (_pid     ) & 0xff;

    unsigned int i;
    for (i=2; i<8; i++)
        _header[i] = rand() & 0xff;

    // initialize payload
    for (i=0; i<_payload_len; i++)
        _payload[i] = rand() & 0xff;
}

// set timespec for timeout
//  _ts         :   pointer to timespec structure
//  _timeout    :   time before timeout
void set_timespec(struct timespec * _ts,
                  float             _timeout)
{
    // get current time (timeval)
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    // add offset (timespec)
    _ts->tv_sec  = tv_now.tv_sec;                       // seconds
    _ts->tv_nsec = tv_now.tv_usec*1000 + _timeout*1e9;  // nanoseconds

    // accumulate nanoseconds into seconds
    while (_ts->tv_nsec > 1000000000) {
        _ts->tv_nsec -= 1000000000;
        _ts->tv_sec++;
    }
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
    if (verbose) {
        int pid         = _header_valid ? (_header[0] << 8 | _header[1]) : -1;
        int payload_len = _header_valid ? _payload_len : -1;
        printf("***** rssi=%7.2fdB evm=%7.2fdB, header:%4s, payload[%6d,%6d bytes]:%4s\n",
                _stats.rssi, _stats.evm,
                _header_valid  ? "pass" : "FAIL",
                pid,
                payload_len,
                _payload_valid ? "pass" : "FAIL");
    }

    // update global counters
    num_frames_detected++;

    if (_header_valid)
        num_valid_headers_received++;

    if (_payload_valid) {
        num_valid_packets_received++;
        num_valid_bytes_received += _payload_len;
    }

#if 0
    // if header was valid, signal condition
    if (_header_valid) {
        pthread_cond_t * rx_cond = (pthread_cond_t*) _userdata;
        pthread_cond_signal(rx_cond);
    }
#endif

    return 0;
}

