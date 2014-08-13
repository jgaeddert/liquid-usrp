/*
 * Copyright (c) 2010 Joseph Gaeddert
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

#include <iostream>
#include <complex>
#include <stdio.h>
#include <stdlib.h>
#include <sys/resource.h>
#include <liquid/liquid.h>
#include <assert.h>

#include <uhd/usrp/multi_usrp.hpp>

#include "timer.h"

void usage() {
    printf("Usage: rssi [OPTION]\n");
    printf("Run receiver, simply printing RSSI to screen periodically\n");
    printf("\n");
    printf("  h     : help\n");
    printf("  v/q   : verbose/quiet\n");
    printf("  f     : center frequency [Hz],   default:  462 MHz\n");
    printf("  b     : bandwidth [Hz],          default:  200 kHz\n");
    printf("  t     : run time [seconds],      default:    5 s\n");
    printf("  G     : uhd rx gain [dB],        default:   20 dB\n");
    printf("  L     : record length [samples], default: 1200 samples\n");
    printf("  o     : output filename,         default: rssi_results.m\n");
}

int main (int argc, char **argv)
{
    // command-line options
    int verbose = true;

    float frequency = 462.0e6;
    float bandwidth = 200e3f;
    float num_seconds = 5.0f;
    double uhd_rxgain = 20.0;

    // output log file
    unsigned int log_size = 1200;
    char filename[256] = "rssi_results.m";

    //
    int d;
    while ((d = getopt(argc,argv,"hvqf:b:t:G:L:o:")) != EOF) {
        switch (d) {
        case 'h':   usage();                        return 0;
        case 'v':   verbose = true;                 break;
        case 'q':   verbose = false;                break;
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 't':   num_seconds = atof(optarg);     break;
        case 'G':   uhd_rxgain = atof(optarg);      break;
        case 'L':   log_size = atoi(optarg);        break;
        case 'o':   strncpy(filename,optarg,255);   break;
        default:
            return 1;
        }
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);

    stream_cmd.stream_now = true;

    uhd::device_addr_t dev_addr;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);

    // try to set hardware rx rate
    usrp->set_rx_rate(2.0f*bandwidth);

    // get actual rx rate
    double usrp_rx_rate = usrp->get_rx_rate();

    // compute arbitrary resampling rate (make up the difference in software)
    double rx_resamp_rate = bandwidth / usrp_rx_rate;

    printf("frequency       :   %10.4f [MHz]\n", frequency*1e-6f);
    printf("bandwidth       :   %10.4f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity       :   %s\n", (verbose?"enabled":"disabled"));
    printf("bandwidth       :   %.4f kHz = %.4f kHz (usrp) * %.4f (resamp rate)\n",
            bandwidth    * 1e-3f,
            usrp_rx_rate * 1e-3f,
            rx_resamp_rate);
    assert(rx_resamp_rate <= 1.0f);

    usrp->set_rx_freq(frequency);
    usrp->set_rx_gain(uhd_rxgain);

    // create and initialize arbitrary resampling component
    msresamp_crcf resamp = msresamp_crcf_create(rx_resamp_rate,60.0f);

    // create automatic gain control object and set properties
    agc_crcf agc_rx = agc_crcf_create();
    agc_crcf_set_bandwidth(agc_rx, 0.01f);

    // create window buffer to log samples/rssi
    windowcf rx_log   = windowcf_create(log_size);
    windowf  rssi_log = windowf_create(log_size);

    //
    const size_t max_samps_per_packet = usrp->get_device()->get_max_recv_samps_per_packet();

    //allocate recv buffer and metatdata
    uhd::rx_metadata_t md;
    std::vector<std::complex<float> > buff(max_samps_per_packet);

    // resampled data (should only be 0 or 1)
    std::complex<float> buffer_resamp[64];

    // start data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    printf("usrp data transfer started\n");
 
    unsigned int i;
    std::complex<float> agc_out;

    // run conditions
    int continue_running = 1;
    timer t0 = timer_create();
    timer_tic(t0);
    float print_runtime = 0.0f;

    while (continue_running) {
        // grab data from port
        size_t num_rx_samps = usrp->get_device()->recv(
            &buff.front(), buff.size(), md,
            uhd::io_type_t::COMPLEX_FLOAT32,
            uhd::device::RECV_MODE_ONE_PACKET
        );

        //handle the error codes
        switch(md.error_code){
        case uhd::rx_metadata_t::ERROR_CODE_NONE:
        case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
            break;

        default:
            std::cerr << "Error code: " << md.error_code << std::endl;
            std::cerr << "Unexpected error on recv, exit test..." << std::endl;
            return 1;
        }

        if (not md.has_time_spec){
            std::cerr << "Metadata missing time spec, exit test..." << std::endl;
            return 1;
        }

        // copy vector "buff" to array of complex float, run
        // resampler, and push through AGC object
        for (i=0; i<num_rx_samps; i++) {
            // push 64 samples into buffer
            std::complex<float> usrp_sample = buff[i];

            // push through resampler (one at a time)
            unsigned int nw;
            msresamp_crcf_execute(resamp, &usrp_sample, 1, buffer_resamp, &nw);

            // push through agc object, push to log buffer
            unsigned int k;
            for (k=0; k<nw; k++) {
                // save time samples
                windowcf_push(rx_log, buffer_resamp[k]);

                // apply agc and get rssi
                agc_crcf_execute(agc_rx, buffer_resamp[k], &agc_out);

                // get linear signal level and push into buffer
                windowf_push(rssi_log, agc_crcf_get_signal_level(agc_rx));
            }
        }

        // check runtime
        float runtime = timer_toc(t0);
        if (runtime >= num_seconds)
            continue_running = 0;
        
        // print rssi to screen
        if ( (runtime - print_runtime) > 0.1f ) {
            print_runtime = runtime;
            printf("rssi : %12.8f dB\n", agc_crcf_get_rssi(agc_rx));
        }
    }
 
    // stop data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    printf("\n");
    printf("usrp data transfer complete\n");

    // clean object allocation
    msresamp_crcf_destroy(resamp);
    agc_crcf_destroy(agc_rx);
    timer_destroy(t0);

    // save log file...
    FILE * fid = fopen(filename,"w");
    if (!fid) {
        fprintf(stderr,"error: %s, could not open '%s' for writing\n", argv[0], filename);
        exit(1);
    }
    fprintf(fid,"%% %s : auto-generated file\n", filename);
    std::complex<float> * rc = NULL;    // complex buffer read pointer
    float * r = NULL;                   // real buffer read pointer
    windowcf_read(rx_log, &rc);
    windowf_read(rssi_log,&r);
    fprintf(fid,"Fs = %e;\n", bandwidth);
    fprintf(fid,"n = %u;\n", log_size);
    fprintf(fid,"rssi = zeros(1,n);\n");
    for (i=0; i<log_size; i++) {
        fprintf(fid,"rssi(%4u) = %12.4e; x(%4u) = %12.4e + %12.4ej;\n",
                i+1, r[i],
                i+1, rc[i].real(), rc[i].imag());
    }
    fprintf(fid,"\n\n");
    fprintf(fid,"t = [0:(n-1)]/Fs*1e3; %% time (ms)\n");
    fprintf(fid,"figure;\n");
    fprintf(fid,"subplot(2,1,1);\n");
    fprintf(fid,"  plot(t,real(x),t,imag(x));\n");
    fprintf(fid,"  ylabel('r(t) [dB]');\n");
    fprintf(fid,"subplot(2,1,2);\n");
    fprintf(fid,"  plot(t,20*log10(rssi));\n");
    fprintf(fid,"  ylabel('RSSI [dB]');\n");
    fprintf(fid,"  xlabel('time [ms]');\n");
    fprintf(fid,"grid on\n");
    fclose(fid);
    printf("output written to '%s'\n", filename);

    windowf_destroy(rssi_log);
    windowcf_destroy(rx_log);

    return 0;
}

