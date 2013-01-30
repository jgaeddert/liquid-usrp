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

#include <iostream>
#include <complex>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <liquid/liquid.h>

#include <uhd/usrp/multi_usrp.hpp>

#include "timer.h"

void usage() {
    printf("Usage: asgram_rx [OPTION]\n");
    printf("Run receiver, printing ascii spectrogram periodically\n");
    printf("\n");
    printf("  h     : help\n");
    printf("  f     : center frequency [Hz], default: 462 MHz\n");
    printf("  b     : bandwidth [Hz],        default: 800 kHz\n");
    printf("  t     : run time [seconds], default: 30\n");
    printf("  G     : uhd rx gain [dB], default: 20 dB\n");
    printf("  n     : FFT size,         default: 64\n");
}

int main (int argc, char **argv)
{
    // command-line options
    int verbose = true;

    float frequency      = 462.0e6;
    float bandwidth      = 800e3f;
    float num_seconds    = 30.0f;
    double uhd_rxgain    = 20.0;
    unsigned int nfft    = 64;
    unsigned int msdelay = 100;

    //
    int d;
    while ((d = getopt(argc,argv,"hf:b:t:G:n:")) != EOF) {
        switch (d) {
        case 'h':   usage();                    return 0;
        case 'f':   frequency   = atof(optarg); break;
        case 'b':   bandwidth   = atof(optarg); break;
        case 't':   num_seconds = atof(optarg); break;
        case 'G':   uhd_rxgain  = atof(optarg); break;
        case 'n':   nfft        = atoi(optarg); break;
        default:    usage();                    return 1;
        }
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

    unsigned int i;

    // set run time appropriately
    if (num_seconds < 0) {
        num_seconds = 1e32; // set to really, really large number
        printf("run time        :   (forever)\n");
    } else {
        printf("run time        :   %f seconds\n", num_seconds);
    }

    // add arbitrary resampling component
    msresamp_crcf resamp = msresamp_crcf_create(rx_resamp_rate, 60.0f);

    // create ASCII spectrogram object
    std::complex<float> x[nfft];
    windowcf buffer = windowcf_create(nfft);
    float maxval;
    float maxfreq;
    char ascii[nfft+1];
    ascii[nfft] = '\0'; // append null character to end of string
    asgram q = asgram_create(x,nfft);
    asgram_set_scale(q,15);
    asgram_set_offset(q,0);

    // assemble footer
    unsigned int footer_len = nfft + 16;
    char footer[footer_len+1];
    for (i=0; i<footer_len; i++)
        footer[i] = ' ';
    footer[1] = '[';
    footer[nfft/2 + 3] = '+';
    footer[nfft + 4] = ']';
    sprintf(&footer[nfft+6], "%8.3f MHz", frequency*1e-6f);
    
    // create/initialize Hamming window
    float w[nfft];
    for (i=0; i<nfft; i++)
        w[i] = hamming(i,nfft);

    unsigned int block_len = 64;
    assert( (block_len % 2) == 0);  // ensure block length is even

    //allocate recv buffer and metatdata
    uhd::rx_metadata_t md;
    const size_t max_samps_per_packet = usrp->get_device()->get_max_recv_samps_per_packet();
    std::vector<std::complex<float> > buff(max_samps_per_packet);

    // create buffer for arbitrary resamper output
    std::complex<float> buffer_resamp[(int)(2.0f/rx_resamp_rate) + 64];
 
    // run conditions
    int continue_running = 1;
    timer t0 = timer_create();
    timer_tic(t0);

    timer t1 = timer_create();
    timer_tic(t1);

    // start data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    printf("usrp data transfer started\n");
 
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
        for (i=0; i<num_rx_samps; i++) {
            // grab sample from usrp buffer
            std::complex<float> usrp_sample = buff[i];

            // push through resampler (one at a time)
            unsigned int nw;
            msresamp_crcf_execute(resamp, &usrp_sample, 1, buffer_resamp, &nw);

            // push resulting samples into buffer
            windowcf_write(buffer, buffer_resamp, nw);
        }

        if (timer_toc(t1) > msdelay*1e-3f) {
            // reset timer
            timer_tic(t1);

            // apply window and run asgram
            std::complex<float> * rc;
            windowcf_read(buffer, &rc);
            for (i=0; i<nfft; i++)
                x[i] = rc[i] * w[i];
            asgram_execute(q, ascii, &maxval, &maxfreq);
            
            // print the spectrogram
            printf(" > %s < pk%5.1fdB [%5.2f]\n", ascii, maxval, maxfreq);
            printf("%s\r", footer);
            fflush(stdout);
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
    msresamp_crcf_destroy(resamp);
    asgram_destroy(q);
    windowcf_destroy(buffer);
    timer_destroy(t0);
    timer_destroy(t1);

    return 0;
}

