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
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <liquid/liquid.h>

#include "ofdmtxrx.h"
#include "timer.h"

void usage() {
    printf("halfduplex_txrx [OPTION]\n");
    printf("transmit OFDM packets back and forth\n");
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
    printf("  t     : rx packet timeout [s],  default:  0.05 s\n");
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
    unsigned int num_frames = 2000;     // number of frames to transmit
    float txgain_dB = -12.0f;           // software tx gain [dB]
    float uhd_txgain = 40.0;            // uhd (hardware) tx gain
    float uhd_rxgain = 20.0;            // uhd (hardware) rx gain

    // ofdm properties
    unsigned int M = 48;                // number of subcarriers
    unsigned int cp_len = 6;            // cyclic prefix length
    unsigned int taper_len = 4;         // taper length

    modulation_scheme ms = LIQUID_MODEM_QPSK;// modulation scheme
    unsigned int payload_len = 1200;        // original data message length
    //crc_scheme check = LIQUID_CRC_32;       // data validity check
    fec_scheme fec0 = LIQUID_FEC_NONE;      // fec (inner)
    fec_scheme fec1 = LIQUID_FEC_GOLAY2412; // fec (outer)

    float timeout   = 0.050;            // timeout (s)
    
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
        case 't':   timeout     = atof(optarg);     break;
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
    } else if (timeout <= 0.0f) {
        fprintf(stderr,"error: %s, ACK timeout must be greater than zero\n", argv[0]);
        exit(-1);
    }

    // thread handling
    pthread_mutex_t rx_mutex;   // receiver mutex
    pthread_cond_t  rx_cond;    // receiver condition
    pthread_mutex_init(&rx_mutex, NULL);
    pthread_cond_init(&rx_cond,   NULL);

    // create transceiver object
    unsigned char * p = NULL;   // default subcarrier allocation
    ofdmtxrx txcvr(M, cp_len, taper_len, p, callback, (void*)&rx_cond);

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
    
    // reset counters
    num_frames_detected=0;
    num_valid_headers_received=0;
    num_valid_packets_received=0;
    num_valid_bytes_received=0;
    
    //
    timer t0 = timer_create();
    timer_tic(t0);
    unsigned int pid;
    //unsigned int i;
    for (pid=0; pid<num_frames; pid++) {
        if (verbose) printf("tx packet id: %6u\n", pid);

        // assemble packet
        assemble_packet(pid, header, payload, payload_len);
        
        // transmit frame
        txcvr.transmit_packet(header, payload, payload_len, ms, fec0, fec1);

        // wait for response or time out; lock mutex
        pthread_mutex_lock(&rx_mutex);
        struct timespec ts;
        set_timespec(&ts, timeout);
        txcvr.start_rx();
        //int status = pthread_cond_wait(&rx_cond, &rx_mutex);
        int status = pthread_cond_timedwait(&rx_cond, &rx_mutex, &ts);
        if (status) printf("timeout\n");
        // unlock the mutex
        pthread_mutex_unlock(&rx_mutex);
        txcvr.stop_rx();
        //txcvr.reset_rx();

    } // packet loop
 
    // sleep for a small amount of time to allow USRP buffers
    // to flush
    usleep(200000);

    //finished
    printf("usrp data transfer complete\n");

    // compute actual run-time
    float runtime = timer_toc(t0);

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
    timer_destroy(t0);
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
        int pid = _header_valid? (_header[0] << 8 | _header[1]) : -1;
        printf("***** rssi=%7.2fdB evm=%7.2fdB, header:%4s, payload[%6d]:%4s\n",
                _stats.rssi, _stats.evm,
                _header_valid  ? "pass" : "FAIL",
                pid,
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

    // if header was valid, signal condition
    if (_header_valid) {
        pthread_cond_t * rx_cond = (pthread_cond_t*) _userdata;
        pthread_cond_signal(rx_cond);
    }

    return 0;
}

