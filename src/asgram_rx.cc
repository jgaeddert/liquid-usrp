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
#include <signal.h>
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
    printf("  G     : uhd rx gain [dB],      default:  20 dB\n");
    printf("  n     : FFT size,              default:  64\n");
    printf("  o     : offset                 default: -65 dB\n");
    printf("  s     : scale                  default:   5 dB\n");
    printf("  r     : FFT rate [Hz],         default:   10 Hz\n");
    printf("  L     : output file log size,  default: 4096 samples\n");
    printf("  F     : output filename,       default: 'asgram_rx.dat'\n");
}

// global running flag
bool continue_running = true;

// signal handler function
void signal_handler(int _signal)
{
    if (_signal == SIGINT) {
        printf("user interrupt\n");
        continue_running = false;
    }
}

// main program
int main (int argc, char **argv)
{
    // command-line options
    int verbose = true;

    float frequency      = 462.0e6;
    float bandwidth      = 800e3f;
    double uhd_rxgain    = 20.0;
    unsigned int nfft    = 64;
    float offset         = -65.0f;
    float scale          = 5.0f;
    float fft_rate       = 10.0f;
    unsigned int logsize = 4096;
    char filename[256]   = "asgram_rx.dat";

    //
    int d;
    while ((d = getopt(argc,argv,"hf:b:G:n:s:o:r:L:F:")) != EOF) {
        switch (d) {
        case 'h':   usage();                    return 0;
        case 'f':   frequency   = atof(optarg); break;
        case 'b':   bandwidth   = atof(optarg); break;
        case 'G':   uhd_rxgain  = atof(optarg); break;
        case 'n':   nfft        = atoi(optarg); break;
        case 'o':   offset      = atof(optarg); break;
        case 's':   scale       = atof(optarg); break;
        case 'r':   fft_rate    = atof(optarg); break;
        case 'L':   logsize     = atoi(optarg); break;
        case 'F':   strncpy(filename,optarg,255); break;
        default:    usage();                    return 1;
        }
    }

    // validate parameters
    if (fft_rate <= 0.0f || fft_rate > 100.0f) {
        fprintf(stderr,"error: %s, fft rate must be in (0, 100) Hz\n", argv[0]);
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

    unsigned int i;

    // add arbitrary resampling component
    msresamp_crcf resamp = msresamp_crcf_create(rx_resamp_rate, 60.0f);

    // create buffer for sample logging
    windowcf log = windowcf_create(logsize);

    // create ASCII spectrogram object
    float maxval;
    float maxfreq;
    char ascii[nfft+1];
    ascii[nfft] = '\0'; // append null character to end of string
    asgramcf q = asgramcf_create(nfft);
    asgramcf_set_scale(q, offset, scale);

    // assemble footer
    unsigned int footer_len = nfft + 16;
    char footer[footer_len+1];
    for (i=0; i<footer_len; i++)
        footer[i] = ' ';
    footer[1] = '[';
    footer[nfft/2 + 3] = '+';
    footer[nfft + 4] = ']';
    sprintf(&footer[nfft+6], "%8.3f MHz", frequency*1e-6f);
    unsigned int msdelay = 1000 / fft_rate;
    
    // create/initialize Hamming window
    //allocate recv buffer and metatdata
    uhd::rx_metadata_t md;
    const size_t max_samps_per_packet = usrp->get_device()->get_max_recv_samps_per_packet();
    std::vector<std::complex<float> > buff(max_samps_per_packet);

    // create buffer for arbitrary resamper output
    std::complex<float> buffer_resamp[(int)(2.0f/rx_resamp_rate) + 64];
 
    // timer to control asgram output
    timer t1 = timer_create();
    timer_tic(t1);

    // start data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    printf("usrp data transfer started\n");

    // catch signal interrupt from user
    if (signal(SIGINT, signal_handler) == SIG_ERR)
        fprintf(stderr,"warning: %s, cannot catch SIGINT\n", argv[0]);

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

            // push resulting samples into asgram object
            asgramcf_write(q, buffer_resamp, nw);

            // write samples to log
            windowcf_write(log, buffer_resamp, nw);
        }

        if (timer_toc(t1) > msdelay*1e-3f) {
            // reset timer
            timer_tic(t1);

            // run the spectrogram
            asgramcf_execute(q, ascii, &maxval, &maxfreq);
            
            // print the spectrogram
            printf(" > %s < pk%5.1fdB [%5.2f]\n", ascii, maxval, maxfreq);
            printf("%s\r", footer);
            fflush(stdout);
        }
    }
 
    // stop data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    printf("\n");
    printf("usrp data transfer complete\n");

    // try to write samples to file
    FILE * fid = fopen(filename,"w");
    if (fid != NULL) {
        // write header
        fprintf(fid, "# %s : auto-generated file\n", filename);
        fprintf(fid, "#\n");
        fprintf(fid, "# num_samples :   %u\n", logsize);
        fprintf(fid, "# frequency   :   %12.8f MHz\n", frequency*1e-6f);
        fprintf(fid, "# bandwidth   :   %12.8f kHz\n", bandwidth*1e-3f);

        // save results to file
        std::complex<float> * rc;   // read pointer
        windowcf_read(log, &rc);
        for (i=0; i<logsize; i++)
            fprintf(fid, "%12.4e %12.4e\n", rc[i].real(), rc[i].imag());

        // close it up
        fclose(fid);
        printf("results written to '%s'\n", filename);
    } else {
        fprintf(stderr,"error: %s, could not open '%s' for writing\n", argv[0], filename);
    }
 
    // destroy objects
    msresamp_crcf_destroy(resamp);
    windowcf_destroy(log);
    asgramcf_destroy(q);
    timer_destroy(t1);

    return 0;
}

