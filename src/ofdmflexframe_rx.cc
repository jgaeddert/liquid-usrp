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

#include <iostream>
#include <complex>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <assert.h>
#include <liquid/liquid.h>

#include <uhd/usrp/multi_usrp.hpp>
 
#include "timer.h"

static bool verbose;

// data counters
unsigned int num_frames_detected;
unsigned int num_valid_headers_received;
unsigned int num_valid_packets_received;
unsigned int num_valid_bytes_received;

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
        // compute true carrier offset
        double samplerate = *((double*)_userdata);
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
    } else {
    }

    // update global counters
    num_frames_detected++;

    if (_header_valid)
        num_valid_headers_received++;

    if (_payload_valid) {
        num_valid_packets_received++;
        num_valid_bytes_received += _payload_len;
    }

    return 0;
}

void usage() {
    printf("ofdmflexframe_rx -- receive OFDM packets\n");
    printf("  u,h   :   usage/help\n");
    printf("  q/v   :   quiet/verbose\n");
    printf("  f     :   center frequency [Hz], default: 462 MHz\n");
    printf("  b     :   bandwidth [Hz], default: 250 kHz\n");
    printf("  G     :   uhd rx gain [dB] (default: 20dB)\n");
    printf("  M     :   number of subcarriers, default: 64\n");
    printf("  C     :   cyclic prefix length, default: 16\n");
    printf("  T     :   taper length, default: 0\n");
    printf("  t     :   run time [seconds]\n");
    printf("  d     :   enable debugging mode\n");
}

int main (int argc, char **argv)
{
    // command-line options
    verbose = true;

    double frequency = 462.0e6;
    double bandwidth = 600e3f;
    double num_seconds = 5.0f;
    double uhd_rxgain = 20.0;

    // ofdm properties
    unsigned int M = 48;                // number of subcarriers
    unsigned int cp_len = 6;            // cyclic prefix length
    unsigned int taper_len = 4;         // taper length

    int debug_enabled =  0;             // enable debugging?

    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:G:M:C:T:t:d")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose = false;                break;
        case 'v':   verbose = true;                 break;
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 'G':   uhd_rxgain = atof(optarg);      break;
        case 'M':   M = atoi(optarg);               break;
        case 'C':   cp_len = atoi(optarg);          break;
        case 'T':   taper_len = atoi(optarg);       break;
        case 't':   num_seconds = atof(optarg);     break;
        case 'd':   debug_enabled = 1;              break;
        default:
            usage();
            return 0;
        }
    }

    if (cp_len == 0 || cp_len > M) {
        fprintf(stderr,"error: %s, cyclic prefix must be in (0,M]\n", argv[0]);
        exit(1);
    }

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);

    stream_cmd.stream_now = true;

    uhd::device_addr_t dev_addr;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);

    // try to set rx rate (oversampled to compensate for CIC filter)
    usrp->set_rx_rate(3.0f * bandwidth);

    // get actual rx rate
    double usrp_rx_rate = usrp->get_rx_rate();

    // compute arbitrary resampling rate (make up the difference in software)
    double rx_resamp_rate = bandwidth / usrp_rx_rate;

    usrp->set_rx_freq(frequency);
    usrp->set_rx_gain(uhd_rxgain);

    printf("frequency       :   %10.4f [MHz]\n", frequency*1e-6f);
    printf("bandwidth       :   %10.4f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity       :   %s\n", (verbose?"enabled":"disabled"));
    printf("usrp sample rate:   %10.4f kHz = %10.4f kHz * %8.6f\n",
            usrp_rx_rate * 1e-3f,
            bandwidth    * 1e-3f,
            1.0f / rx_resamp_rate);

    // set run time appropriately
    if (num_seconds < 0) {
        num_seconds = 1e32; // set to really, really large number
        printf("run time        :   (forever)\n");
    } else {
        printf("run time        :   %f seconds\n", num_seconds);
    }

    // add arbitrary resampling component
    msresamp_crcf resamp = msresamp_crcf_create(rx_resamp_rate, 60.0f);

    unsigned int block_len = 64;
    assert( (block_len % 2) == 0);  // ensure block length is even

    //allocate recv buffer and metatdata
    uhd::rx_metadata_t md;
    const size_t max_samps_per_packet = usrp->get_device()->get_max_recv_samps_per_packet();
    std::vector<std::complex<float> > buff(max_samps_per_packet);

    // create frame synchronizer (default subcarrier allocation)
    ofdmflexframesync fs = ofdmflexframesync_create(M,cp_len,taper_len,NULL,callback,(void*)&bandwidth);
    if (debug_enabled)
        ofdmflexframesync_debug_enable(fs);
    ofdmflexframesync_print(fs);

    // start data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    printf("usrp data transfer started\n");
 
    // create buffer for arbitrary resamper output
    std::complex<float> buffer_resamp[(int)(2.0f/rx_resamp_rate) + 64];
 
    // reset counters
    num_frames_detected=0;
    num_valid_headers_received=0;
    num_valid_packets_received=0;
    num_valid_bytes_received=0;

    // run conditions
    int continue_running = 1;
    timer t0 = timer_create();
    timer_tic(t0);

    while (continue_running) {
        // grab data from device
        size_t num_rx_samps = usrp->get_device()->recv(
            &buff.front(), buff.size(), md,
            uhd::io_type_t::COMPLEX_FLOAT32,
            uhd::device::RECV_MODE_ONE_PACKET
        );

        // 'handle' the error codes
        switch(md.error_code){
        case uhd::rx_metadata_t::ERROR_CODE_NONE:
        case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
            break;

        default:
            std::cerr << "Error code: " << md.error_code << std::endl;
            std::cerr << "Unexpected error on recv, exit test..." << std::endl;
            return 1;
        }

        // push data through arbitrary resampler and give to frame synchronizer
        // TODO : apply bandwidth-dependent gain
        unsigned int j;
        for (j=0; j<num_rx_samps; j++) {
            // grab sample from usrp buffer
            std::complex<float> usrp_sample = buff[j];

            // push through resampler (one at a time)
            unsigned int nw;
            msresamp_crcf_execute(resamp, &usrp_sample, 1, buffer_resamp, &nw);

            // push resulting samples through synchronizer
            ofdmflexframesync_execute(fs, buffer_resamp, nw);
        }

        // check runtime
        if (timer_toc(t0) >= num_seconds)
            continue_running = 0;
    }
 
    // compute actual run-time
    float runtime = timer_toc(t0);

    // stop data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    printf("\n");
    printf("usrp data transfer complete\n");
 
    // print results
    float data_rate = num_valid_bytes_received * 8.0f / num_seconds;
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

    // export debugging file
    if (debug_enabled)
        ofdmflexframesync_debug_print(fs, "ofdmflexframesync_debug.m");

    // destroy objects
    msresamp_crcf_destroy(resamp);
    ofdmflexframesync_destroy(fs);
    timer_destroy(t0);

    return 0;
}

