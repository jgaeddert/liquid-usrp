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

#include <uhd/usrp/single_usrp.hpp>

void usage() {
    printf("Usage: rssi [OPTION]\n");
    printf("Run receiver, simply printing RSSI to screen periodically\n");
    printf("\n");
    printf("  f     : center frequency [Hz]\n");
    printf("  b     : bandwidth [Hz]\n");
    printf("  t     : run time [seconds]\n");
    printf("  G     : uhd rx gain [dB] (default: 20dB)\n");
    printf("  q     : quiet\n");
    printf("  v     : verbose\n");
    printf("  L     : record length (number of samples), default: 1200\n");
    printf("  o     : output filename\n");
    printf("  u,h   : usage/help\n");
}

int main (int argc, char **argv)
{
    // command-line options
    int verbose = true;

    unsigned long int ADC_RATE = 64e6;
    float min_bandwidth = 0.25f*(ADC_RATE / 256.0);
    float max_bandwidth = 0.25f*(ADC_RATE /   4.0);

    float frequency = 462.0e6;
    float bandwidth = 100e3f;
    float num_seconds = 5.0f;
    double uhd_rxgain = 20.0;

    // output log file
    unsigned int log_size = 1200;
    char filename[256] = "rssi_results.m";

    //
    int d;
    while ((d = getopt(argc,argv,"f:b:t:G:qvL:o:uh")) != EOF) {
        switch (d) {
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 't':   num_seconds = atof(optarg);     break;
        case 'G':   uhd_rxgain = atof(optarg);      break;
        case 'q':   verbose = false;                break;
        case 'v':   verbose = true;                 break;
        case 'L':   log_size = atoi(optarg);        break;
        case 'o':   strncpy(filename,optarg,255);   break;
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

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);

    stream_cmd.stream_now = true;

    uhd::device_addr_t dev_addr;
    uhd::usrp::single_usrp::sptr usrp = uhd::usrp::single_usrp::make(dev_addr);

    // set properties
    float rx_rate = 4.0f*bandwidth;
#if 0
    usrp->set_rx_rate(rx_rate);
#else
    // NOTE : the sample rate computation MUST be in double precision so
    //        that the UHD can compute its decimation rate properly
    unsigned int decim_rate = (unsigned int)(ADC_RATE / rx_rate);
    // ensure multiple of 2
    decim_rate = (decim_rate >> 1) << 1;
    // compute usrp sampling rate
    double usrp_rx_rate = ADC_RATE / decim_rate;
    // compute arbitrary resampling rate
    double rx_resamp_rate = rx_rate / usrp_rx_rate;
    printf("sample rate :   %12.8f kHz = %12.8f * %8.6f (decim %u)\n",
            rx_rate * 1e-3f,
            usrp_rx_rate * 1e-3f,
            rx_resamp_rate,
            decim_rate);
    usrp->set_rx_rate(ADC_RATE / decim_rate);
#endif
    usrp->set_rx_freq(frequency);
    usrp->set_rx_gain(uhd_rxgain);

    // create and initialize arbitrary resampling component
    resamp_crcf resamp = resamp_crcf_create(rx_resamp_rate,7,0.4f,60.0f,64);
    resamp_crcf_setrate(resamp, rx_resamp_rate);

    // create automatic gain control object and set properties
    agc_crcf agc_rx = agc_crcf_create();
    agc_crcf_set_bandwidth(agc_rx, 0.01f);
    agc_crcf_set_gain_limits(agc_rx, 1e-4f, 1e4f);

    // create window buffer to log samples
    windowf log_buffer = windowf_create(log_size);

    //
    const size_t max_samps_per_packet = usrp->get_device()->get_max_recv_samps_per_packet();
    unsigned int num_blocks = (unsigned int)((rx_rate*num_seconds)/(max_samps_per_packet));

    //allocate recv buffer and metatdata
    uhd::rx_metadata_t md;
    std::vector<std::complex<float> > buff(max_samps_per_packet);

    std::complex<float> data_rx[64];        // received data straight from USRP
    std::complex<float> data_resamp[200];   // resampled data

    // start data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    printf("usrp data transfer started\n");
 
    unsigned int i;
    unsigned int k;
    unsigned int n=0;   // sample counter
    std::complex<float> agc_out;

    for (i=0; i<num_blocks; i++) {
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
        unsigned int j;
        unsigned int nw=0;
        for (j=0; j<num_rx_samps; j++) {
            // push 64 samples into buffer
            data_rx[n++] = buff[j];

            if (n==64) {
                // reset counter
                n=0;

                // apply resampler
                unsigned int num_resamp=0;
                for (k=0; k<64; k++) {
                    resamp_crcf_execute(resamp, data_rx[k], &data_resamp[num_resamp], &nw);
                    num_resamp += nw;
                }

                // push through agc object, push to log buffer
                for (k=0; k<num_resamp; k++) {
                    agc_crcf_execute(agc_rx, data_resamp[k], &agc_out);

                    // get linear signal level and push into buffer
                    windowf_push(log_buffer, agc_crcf_get_signal_level(agc_rx));
                }
            }

        }

        // print rssi to screen
        if ( (i%10)==0 )
            printf("rssi : %12.8f dB\n", agc_crcf_get_rssi(agc_rx));

    }
 
    // stop data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    printf("\n");
    printf("usrp data transfer complete\n");

    // clean object allocation
    resamp_crcf_destroy(resamp);
    agc_crcf_destroy(agc_rx);

    // save log file...
    FILE * fid = fopen(filename,"w");
    if (!fid) {
        fprintf(stderr,"error: %s, could not open '%s' for writing\n", argv[0], filename);
        exit(1);
    }
    fprintf(fid,"%% %s : auto-generated file\n", filename);
    float * r = NULL;
    windowf_read(log_buffer,&r);
    fprintf(fid,"Fs = %e;\n", rx_rate);
    fprintf(fid,"n = %u;\n", log_size);
    fprintf(fid,"rssi = zeros(1,n);\n");
    for (i=0; i<log_size; i++)
        fprintf(fid,"rssi(%u) = %12.4e;\n", i+1, r[i]);
    fprintf(fid,"\n\n");
    fprintf(fid,"t = [0:(n-1)]/Fs;\n");
    fprintf(fid,"figure;\n");
    fprintf(fid,"plot(t*1e3,10*log10(rssi));\n");
    fprintf(fid,"xlabel('time [ms]');\n");
    fprintf(fid,"ylabel('RSSI [dB]');\n");
    fprintf(fid,"grid on\n");
    fclose(fid);
    printf("output written to '%s'\n", filename);

    windowf_destroy(log_buffer);

    return 0;
}

