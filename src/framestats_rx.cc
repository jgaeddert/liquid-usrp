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

//
// framestats_tx.cc
//
// frame statistics transmitter
//

#include <iostream>
#include <complex>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <liquid/liquid.h>

#include "usrp_io.h"
 
#define USRP_CHANNEL        (0)
 
static bool verbose;

static float SNRdB_av;

unsigned int num_steps = 36;
float SNRdB_min =  -5.0f;
float SNRdB_max =  30.0f;
float * SNRdB;
static unsigned int * num_packets_received;
static unsigned int * num_valid_headers_received;
static unsigned int * num_valid_packets_received;
static unsigned int * num_bytes_received;

typedef struct {
    unsigned char * header;
    unsigned char * payload;
    packetizer p;
} framedata;

static int callback(unsigned char * _rx_header,
                    int _rx_header_valid,
                    unsigned char * _rx_payload,
                    unsigned int _rx_payload_len,
                    framesyncstats_s _stats,
                    void * _userdata)
{
    // estimate SNR
    float noise_floor = -38.0f;
    float SNRdB_hat = _stats.rssi - noise_floor;

    // compute array index
    int i = 0;
    float SNRdB_step = (SNRdB_max - SNRdB_min) / (float)(num_steps-1);
    i = (int) floorf( (SNRdB_hat - SNRdB_min) / SNRdB_step );
    if (i < 0) {
        fprintf(stderr,"warning: SNR estimate below boundary (low)\n");
        i = 0;
    } else if (i > (int)num_steps-1) {
        fprintf(stderr,"warning: SNR estimate below boundary (high)\n");
        i = num_steps-1;
    }
    //printf("  SNR: %6.2f  ->  %d\n", SNRdB_hat, i);

    num_packets_received[i]++;
    if (verbose) {
        printf("********* callback invoked, ");
        printf("SNR=%5.1fdB, ", _stats.SNR);
        printf("rssi=%5.1fdB, ", _stats.rssi);
    }

    // make better estimate of SNR
    SNRdB_av += SNRdB_hat;

    if ( !_rx_header_valid ) {
        if (verbose) printf("header crc : FAIL\n");
        return 0;
    }
    num_valid_headers_received[i]++;
    unsigned int packet_id = (_rx_header[0] << 8 | _rx_header[1]);
    if (verbose) printf("packet id: %6u\n", packet_id);
    unsigned int payload_len = (_rx_header[2] << 8 | _rx_header[3]);
    fec_scheme fec0 = (fec_scheme)(_rx_header[4]);
    fec_scheme fec1 = (fec_scheme)(_rx_header[5]);

    /*
    // TODO: validate fec0,fec1 before indexing fec_scheme_str
    printf("    payload len : %u\n", payload_len);
    printf("    fec0        : %s\n", fec_scheme_str[fec0]);
    printf("    fec1        : %s\n", fec_scheme_str[fec1]);
    */

    framedata * fd = (framedata*) _userdata;
    fd->p = packetizer_recreate(fd->p, payload_len, fec0, fec1);

    // decode packet
    unsigned char msg_dec[payload_len];
    bool crc_pass = packetizer_decode(fd->p, _rx_payload, msg_dec);
    if (crc_pass) {
        num_valid_packets_received[i]++;
        num_bytes_received[i] += payload_len;
    } else {
        if (verbose) printf("  <<< payload crc fail >>>\n");
    }

    /*
    packetizer_print(fd->p);
    printf("payload len: %u\n", _rx_payload_len);
    unsigned int i;
    for (i=0; i<_rx_payload_len; i++)
        printf("%.2x ", _rx_payload[i]);
    printf("\n");

    for (i=0; i<payload_len; i++)
        printf("%.2x ", msg_dec[i]);
    printf("\n");
    */

    return 0;
}

double calculate_execution_time(struct rusage _start, struct rusage _finish)
{
    double utime = _finish.ru_utime.tv_sec - _start.ru_utime.tv_sec
        + 1e-6*(_finish.ru_utime.tv_usec - _start.ru_utime.tv_usec);
    double stime = _finish.ru_stime.tv_sec - _start.ru_stime.tv_sec
        + 1e-6*(_finish.ru_stime.tv_usec - _start.ru_stime.tv_usec);
    //printf("utime : %12.8f, stime : %12.8f\n", utime, stime);
    return utime + stime;
}

void usage() {
    printf("packet_tx:\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  b     :   bandwidth [Hz]\n");
    printf("  t     :   run time [seconds]\n");
    printf("  o     :   output filename, default: framestats.dat\n");
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

    char filename[256] = "framestats.dat";

    //
    int d;
    while ((d = getopt(argc,argv,"f:b:t:o:qvuh")) != EOF) {
        switch (d) {
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 't':   num_seconds = atof(optarg);     break;
        case 'o':   strncpy(filename,optarg,256);   break;
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

    // initialize arrays
    SNRdB = (float *) malloc(num_steps * sizeof(float));
    num_packets_received        = (unsigned int *) malloc(num_steps * sizeof(unsigned int));
    num_valid_headers_received  = (unsigned int *) malloc(num_steps * sizeof(unsigned int));
    num_valid_packets_received  = (unsigned int *) malloc(num_steps * sizeof(unsigned int));
    num_bytes_received          = (unsigned int *) malloc(num_steps * sizeof(unsigned int));

    unsigned int i;
    float SNRdB_step = (SNRdB_max - SNRdB_min) / (float)(num_steps - 1);
    for (i=0; i<num_steps; i++) {
        SNRdB[i] = SNRdB_min + i*SNRdB_step;
        //printf("    %3u : %6.2f\n", i, SNRdB[i]);

        num_packets_received[i] = 0;
        num_valid_headers_received[i] = 0;
        num_valid_packets_received[i] = 0;
        num_bytes_received[i] = 0;
    }

    unsigned int num_blocks = (unsigned int)((2.0f*bandwidth*num_seconds)/(512));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_rx_freq(USRP_CHANNEL, frequency);
    uio->set_rx_samplerate(2.0f*bandwidth);
    uio->enable_auto_tx(USRP_CHANNEL);

    // retrieve rx port
    gport port_rx = uio->get_rx_port(USRP_CHANNEL);

    SNRdB_av = 0.0f;

    // framing
    packetizer p = packetizer_create(0,FEC_NONE,FEC_NONE);
    framedata fd;
    fd.header = NULL;
    fd.payload = NULL;
    fd.p = p;
    // set properties to default
    framesyncprops_s props;
    framesyncprops_init_default(&props);
    props.squelch_threshold = -37.0f;
    props.squelch_enabled = 0;
#if 0
    props.agc_bw0 = 1e-3f;
    props.agc_bw1 = 1e-5f;
    props.agc_gmin = 1e-3f;
    props.agc_gmax = 1e4f;
    props.pll_bw0 = 1e-3f;
    props.pll_bw1 = 3e-5f;
#endif
    flexframesync fs = flexframesync_create(&props,callback,(void*)&fd);

    std::complex<float> data_rx[512];
 
    // start usrp data transfer
    uio->start_rx(USRP_CHANNEL);
    printf("usrp data transfer started\n");
 
    struct rusage start;
    struct rusage finish;
    getrusage(RUSAGE_SELF, &start);
    // read samples from buffer, run through frame synchronizer
    for (i=0; i<num_blocks; i++) {
        // grab data from port
        gport_consume(port_rx, (void*)data_rx, 512);

        // run through frame synchronizer
        flexframesync_execute(fs, data_rx, 512);
    }
    getrusage(RUSAGE_SELF, &finish);

    // stop usrp data transfer
    uio->stop_rx(USRP_CHANNEL);
    printf("usrp data transfer complete\n");

    // clean up allocated objects
    flexframesync_destroy(fs);
    packetizer_destroy(p);
    delete uio;

#if 0
    // print results
    double extime = calculate_execution_time(start,finish);
    float data_rate = 8.0f * (float)(num_bytes_received) / num_seconds;
    float percent_headers_valid = (num_packets_received == 0) ?
                          0.0f :
                          100.0f * (float)num_valid_headers_received / (float)num_packets_received;
    float percent_packets_valid = (num_packets_received == 0) ?
                          0.0f :
                          100.0f * (float)num_valid_packets_received / (float)num_packets_received;
    if (num_packets_received > 0)
        SNRdB_av /= num_packets_received;
    float PER = 1.0f - 0.01f*percent_packets_valid;
    float spectral_efficiency = data_rate / bandwidth;
    printf("    packets received    : %6u\n", num_packets_received);
    printf("    valid headers       : %6u (%6.2f%%)\n", num_valid_headers_received,percent_headers_valid);
    printf("    valid packets       : %6u (%6.2f%%)\n", num_valid_packets_received,percent_packets_valid);
    printf("    packet error rate   : %16.8e\n", PER);
    printf("    average SNR [dB]    : %8.4f\n", SNRdB_av);
    printf("    bytes_received      : %6u\n", num_bytes_received);
    printf("    data rate           : %12.8f kbps\n", data_rate*1e-3f);
    printf("    spectral efficiency : %12.8f b/s/Hz\n", spectral_efficiency);
    printf("    execution time      : %12.8f s\n", extime);
    printf("    %% cpu               : %12.8f\n", 100.0f*extime / num_seconds);

    printf("    # rx   # ok      %% ok        # data     %% data       PER          SNR      sp. eff.\n");
    printf("    %-6u %-6u  %12.4e  %-6u   %12.4e %12.4e  %8.4f  %6.4f\n",
            num_packets_received,
            num_valid_headers_received,
            percent_headers_valid * 0.01f,
            num_valid_packets_received,
            percent_packets_valid * 0.01f,
            PER,
            SNRdB_av,
            spectral_efficiency);
#else
    printf("    SNR [dB]    detected    headers     payloads    rate [kbps]\n");
    for (i=0; i<num_steps; i++) {
        float rate = (num_valid_packets_received[i] == 0) ? 0 :
            num_bytes_received[i] * 8.0f * 1e-3f;
        printf("    %6.2f      %-6u      %-6u      %-6u    %8.3f\n",
               SNRdB[i],
               num_packets_received[i],
               num_valid_headers_received[i],
               num_valid_packets_received[i],
               rate);
    }
#endif
    // save results to output file
    FILE * fid = fopen(filename,"w");
    if (!fid) {
        fprintf(stderr,"error: could not open '%s' for writing\n", filename);
        exit(1);
    }
    fprintf(fid,"# framestats data\n");
    fprintf(fid,"#   SNR [dB]    detected    headers     payloads    rate [kbps]\n");
    for (i=0; i<num_steps; i++) {
        float rate = (num_valid_packets_received[i] == 0) ? 0 :
            num_bytes_received[i] * 8.0f * 1e-3f;
        fprintf(fid,"    %6.2f      %-6u      %-6u      %-6u    %12.4e\n",
                SNRdB[i],
                num_packets_received[i],
                num_valid_headers_received[i],
                num_valid_packets_received[i],
                rate);
    }

    fclose(fid);
    printf("results written to '%s'\n", filename);

    // free memory arrays
    free(SNRdB);
    free(num_packets_received);
    free(num_valid_headers_received);
    free(num_valid_packets_received);
    free(num_bytes_received);

    return 0;
}

