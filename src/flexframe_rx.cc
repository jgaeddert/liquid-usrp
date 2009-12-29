/*
 * Copyright (c) 2007, 2009 Joseph Gaeddert
 * Copyright (c) 2007, 2009 Virginia Polytechnic Institute & State University
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
#include <liquid/liquid.h>

#include "usrp_io.h"
 
#define USRP_CHANNEL        (0)
 
static bool verbose;
static unsigned int num_packets_received;
static unsigned int num_valid_headers_received;
static unsigned int num_valid_packets_received;
static unsigned int num_bytes_received;

typedef struct {
    unsigned char * header;
    unsigned char * payload;
    packetizer p;
} framedata;

static int callback(unsigned char * _rx_header,
                    int _rx_header_valid,
                    unsigned char * _rx_payload,
                    unsigned int _rx_payload_len,
                    void * _userdata)
{
    num_packets_received++;
    if (verbose) printf("********* callback invoked, ");

    if ( !_rx_header_valid ) {
        if (verbose) printf("header crc : FAIL\n");
        return 0;
    }
    num_valid_headers_received++;
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
        num_valid_packets_received++;
        num_bytes_received += payload_len;
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

    unsigned int num_blocks = (unsigned int)((4.0f*bandwidth*num_seconds)/(512));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_rx_freq(USRP_CHANNEL, frequency);
    uio->set_rx_decim(decim_rate);
    uio->enable_auto_tx(USRP_CHANNEL);

    // retrieve rx port
    gport port_rx = uio->get_rx_port(USRP_CHANNEL);

    num_packets_received = 0;
    num_valid_packets_received = 0;
    num_valid_headers_received = 0;
    num_bytes_received = 0;

    // framing
    packetizer p = packetizer_create(0,FEC_NONE,FEC_NONE);
    framedata fd;
    fd.header = NULL;
    fd.payload = NULL;
    fd.p = p;
    // set properties to default
    flexframesyncprops_s props;
    flexframesyncprops_init_default(&props);
    props.squelch_threshold = -32.0f;
    props.agc_bw0 = 1e-3f;
    props.agc_bw1 = 1e-5f;
    props.agc_gmin = 1e-3f;
    props.agc_gmax = 1e4f;
    props.pll_bw0 = 1e-3f;
    props.pll_bw1 = 3e-5f;
    flexframesync fs = flexframesync_create(&props,callback,(void*)&fd);

    // create decimator
    resamp2_crcf decimator = resamp2_crcf_create(37,0.0f,60.0f);
    std::complex<float> decim_out[256];

    std::complex<float> * data_rx;
 
    // start usrp data transfer
    uio->start_rx(USRP_CHANNEL);
    printf("usrp data transfer started\n");
 
    unsigned int i, n;
    // read samples from buffer, run through frame synchronizer
    for (i=0; i<num_blocks; i++) {
        // grab data from port
        data_rx = (std::complex<float>*) gport_consumer_lock(port_rx,512);

        // run decimator
        for (n=0; n<256; n++) {
            resamp2_crcf_decim_execute(decimator, &data_rx[2*n], &decim_out[n]);
        }

        // run through frame synchronizer
        flexframesync_execute(fs, decim_out, 256);

        // release port
        gport_consumer_unlock(port_rx,512);
    }
 
    // stop usrp data transfer
    uio->stop_rx(USRP_CHANNEL);
    printf("usrp data transfer complete\n");

    // print results
    float data_rate = 8.0f * (float)(num_bytes_received) / num_seconds;
    float percent_headers_valid = (num_packets_received == 0) ?
                          0.0f :
                          100.0f * (float)num_valid_headers_received / (float)num_packets_received;
    float percent_packets_valid = (num_packets_received == 0) ?
                          0.0f :
                          100.0f * (float)num_valid_packets_received / (float)num_packets_received;
    printf("    packets received    : %6u\n", num_packets_received);
    printf("    valid headers       : %6u (%6.2f%%)\n", num_valid_headers_received,percent_headers_valid);
    printf("    valid packets       : %6u (%6.2f%%)\n", num_valid_packets_received,percent_packets_valid);
    printf("    bytes_received      : %6u\n", num_bytes_received);
    printf("    data rate           : %12.8f kbps\n", data_rate*1e-3f);

    // clean it up
    flexframesync_destroy(fs);
    packetizer_destroy(p);
    resamp2_crcf_destroy(decimator);

    delete uio;
    return 0;
}

