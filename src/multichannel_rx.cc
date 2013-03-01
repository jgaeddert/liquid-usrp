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
#include "multichannelrx.h"

static bool verbose;

// global callback function
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
        printf("***** rssi=%7.2fdB evm=%7.2fdB, ", _stats.rssi, _stats.evm);

        if (_header_valid) {
            unsigned int packet_id = (_header[0] << 8 | _header[1]);
	    unsigned int channel = _header[2];
	    printf("channel: %u ", channel);
            printf("rx packet id: %6u", packet_id);
            if (_payload_valid) 
	    {
	      printf("\n");
	    } 
            else                printf(" PAYLOAD INVALID\n");
        } else {
            printf("HEADER INVALID\n");
        }
    } else {
    }

    return 0;
}
void usage() {
    printf("ofdmflexframe_rx -- receive OFDM packets\n");
    printf("  u,h   : usage/help\n");
    printf("  q/v   : quiet/verbose\n");
    printf("  f     : center frequency [Hz], default: 462 MHz\n");
    printf("  b     : bandwidth [Hz],        default: 250 kHz\n");
    printf("  M     : number of subcarriers, default:  48\n");
    printf("  C     : cyclic prefix length,  default:   6\n");
    printf("  T     : taper length,          default:   4\n");
    printf("  n     : number of channels,    default: 1\n");
    printf("  G     : uhd rx gain [dB],      default: 20 dB\n");
    printf("  t     : run time [seconds],    default: 10\n");
}

int main (int argc, char **argv)
{
    // command-line options
    verbose = true;

    double frequency = 462.0e6;         // center frequency [Hz]
    double bandwidth = 250e3f;          // channel bandwidth [Hz]
    unsigned int num_channels = 1;      // number of channels
    double num_seconds = 10.0f;         // run time
    double uhd_rxgain = 20.0;           // uhd (hardware) rx gain

    // ofdm properties
    unsigned int M          = 48;       // number of subcarriers
    unsigned int cp_len     =  6;       // cyclic prefix length
    unsigned int taper_len  =  4;       // cyclic prefix length

    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:M:C:T:n:G:t:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose     = false;            break;
        case 'v':   verbose     = true;             break;
        case 'f':   frequency   = atof(optarg);     break;
        case 'b':   bandwidth   = atof(optarg);     break;
        case 'M':   M           = atoi(optarg);     break;
        case 'C':   cp_len      = atoi(optarg);     break;
        case 'T':   taper_len   = atoi(optarg);     break;
        case 'n':   num_channels= atoi(optarg);     break;
        case 'G':   uhd_rxgain  = atof(optarg);     break;
        case 't':   num_seconds = atof(optarg);     break;
        default:
            usage();
            return 0;
        }
    }

    unsigned int i;

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);

    stream_cmd.stream_now = true;

    uhd::device_addr_t dev_addr;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);

    // set properties
    double rx_rate = num_channels*bandwidth;

    // try to set rx rate (oversampled to compensate for CIC filter)
    usrp->set_rx_rate(2.0f * rx_rate);

    // get actual rx rate
    double usrp_rx_rate = usrp->get_rx_rate();

    // compute arbitrary resampling rate (make up the difference in software)
    double rx_resamp_rate = rx_rate / usrp_rx_rate;

    usrp->set_rx_freq(frequency);
    usrp->set_rx_gain(uhd_rxgain);

    printf("frequency       :   %10.4f [MHz]\n", frequency*1e-6f);
    printf("channels        :   %10.4f kHz x %u\n", bandwidth * 1e-3f, num_channels);
    printf("usrp sample rate:   %10.4f kHz = %10.4f kHz * %8.6f\n",
            usrp_rx_rate * 1e-3f,
            rx_rate      * 1e-3f,
            1.0f / rx_resamp_rate);
    printf("verbosity       :   %s\n", (verbose?"enabled":"disabled"));


    if (num_seconds >= 0)
        printf("run time    :   %f seconds\n", num_seconds);
    else
        printf("run time    :   (forever)\n");

    unsigned int block_len = 64;
    assert( (block_len % 2) == 0);  // ensure block length is even

    //allocate recv buffer and metatdata
    uhd::rx_metadata_t md;
    const size_t max_samps_per_packet = usrp->get_device()->get_max_recv_samps_per_packet();
    std::vector<std::complex<float> > buff(max_samps_per_packet);

    // create multi-channel receiver object
    void * userdata[num_channels];
    framesync_callback callbacks[num_channels];
    for (i=0; i<num_channels; i++) {
        userdata[i] = NULL;
        callbacks[i] = callback;
    }
    unsigned char * p = NULL;   // default subcarrier allocation
    multichannelrx mcrx(num_channels, M, cp_len, taper_len, p, userdata, callbacks);
    
    // start data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    printf("usrp data transfer started\n");
 
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
        unsigned int j;
        for (j=0; j<num_rx_samps; j++) {
            // grab sample from usrp buffer
            std::complex<float> usrp_sample = buff[j];

            // push resulting samples through receiver
            mcrx.Execute(&usrp_sample, 1);
        }

        // check runtime
        if (timer_toc(t0) >= num_seconds)
            continue_running = 0;
    }
 
    // stop data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    printf("\n");
    printf("usrp data transfer complete\n");
 
    // destroy objects
    timer_destroy(t0);

    return 0;
}

