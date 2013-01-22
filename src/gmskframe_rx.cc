/*
 * Copyright (c) 2011 Joseph Gaeddert
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

#include <uhd/usrp/multi_usrp.hpp>
 
#include "timer.h"

static bool verbose;
static unsigned int num_packets_received;
static unsigned int num_valid_headers_received;
static unsigned int num_valid_packets_received;
static unsigned int num_bytes_received;

static float SNRdB_av;

static int callback(unsigned char *  _header,
                    int              _header_valid,
                    unsigned char *  _payload,
                    unsigned int     _payload_len,
                    int              _payload_valid,
                    framesyncstats_s _stats,
                    void *           _userdata)
{
    num_packets_received++;
    if (verbose) {
        printf("********* callback invoked, ");
        printf("evm=%5.1fdB, ", _stats.evm);
        printf("rssi=%5.1fdB, ", _stats.rssi);
    }

    // make better estimate of SNR
    float noise_floor = -38.0f; // noise floor estimate
    float SNRdB = _stats.rssi - noise_floor;
    SNRdB_av += SNRdB;

    if ( !_header_valid ) {
        if (verbose) printf("header crc : FAIL\n");
        return 0;
    }
    num_valid_headers_received++;
    unsigned int packet_id = (_header[0] << 8 | _header[1]);
    if (verbose) printf("packet id: %6u\n", packet_id);

    if ( !_payload_valid ) {
        if (verbose) printf("payload crc : FAIL\n");
        return 0;
    }

    num_valid_packets_received++;
    num_bytes_received += _payload_len;

    return 0;
}

void usage() {
    printf("gmskframe_tx:\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  b     :   bandwidth [Hz]\n");
    printf("  t     :   run time [seconds]\n");
    printf("  G     :   uhd rx gain [dB] (default: 20dB)\n");
    printf("  q     :   quiet\n");
    printf("  v     :   verbose\n");
    printf("  u,h   :   usage/help\n");
}

int main (int argc, char **argv)
{
    // command-line options
    verbose = true;
    unsigned long int ADC_RATE = 64e6;

    float min_bandwidth = 0.25f*(ADC_RATE / 256.0);
    float max_bandwidth = 0.25f*(ADC_RATE /   4.0);

    float frequency = 462.0e6;
    float bandwidth = 100e3;
    float num_seconds = 5.0f;
    double uhd_rxgain = 20.0;

    //
    int d;
    while ((d = getopt(argc,argv,"f:b:t:G:qvuh")) != EOF) {
        switch (d) {
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 't':   num_seconds = atof(optarg);     break;
        case 'G':   uhd_rxgain = atof(optarg);      break;
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

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);

    stream_cmd.stream_now = true;

    uhd::device_addr_t dev_addr;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);

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
    
    // try to set rx rate
    usrp->set_rx_rate(ADC_RATE / decim_rate);

    // get actual rx rate
    usrp_rx_rate = usrp->get_rx_rate();

    // compute arbitrary resampling rate
    double rx_resamp_rate = rx_rate / usrp_rx_rate;
    printf("sample rate :   %12.8f kHz = %12.8f * %8.6f (decim %u)\n",
            rx_rate * 1e-3f,
            usrp_rx_rate * 1e-3f,
            rx_resamp_rate,
            decim_rate);
#endif
    usrp->set_rx_freq(frequency);
    usrp->set_rx_gain(uhd_rxgain);

    // add arbitrary resampling component
    resamp_crcf resamp = resamp_crcf_create(rx_resamp_rate,7,0.4f,60.0f,64);
    resamp_crcf_setrate(resamp, rx_resamp_rate);

    // half-band resampler
    resamp2_crcf decim = resamp2_crcf_create(7,0.0f,40.0f);

    const size_t max_samps_per_packet = usrp->get_device()->get_max_recv_samps_per_packet();

    //allocate recv buffer and metatdata
    uhd::rx_metadata_t md;
    std::vector<std::complex<float> > buff(max_samps_per_packet);

    num_packets_received = 0;
    num_valid_packets_received = 0;
    num_valid_headers_received = 0;
    num_bytes_received = 0;
    SNRdB_av = 0.0f;

    // create frame synchronizer
    gmskframesync fs = gmskframesync_create(callback,NULL);

    std::complex<float> data_rx[64];
    std::complex<float> data_decim[32];
    std::complex<float> data_resamp[64];

    // start data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    printf("usrp data transfer started\n");
 
    // run conditions
    int continue_running = 1;
    timer t0 = timer_create();
    timer_tic(t0);

    unsigned int n=0;
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

        // for now copy vector "buff" to array of complex float
        // TODO : apply bandwidth-dependent gain
        unsigned int j;
        unsigned int nw=0;
        for (j=0; j<num_rx_samps; j++) {
            // push 64 samples into buffer
            data_rx[n++] = buff[j];

            if (n==64) {
                // reset counter
                n=0;

                // deimate to 32
                unsigned int k;
                for (k=0; k<32; k++)
                    resamp2_crcf_decim_execute(decim, &data_rx[2*k], &data_decim[k]);

                // apply resampler
                for (k=0; k<32; k++) {
                    resamp_crcf_execute(resamp, data_decim[k], &data_resamp[n], &nw);
                    n += nw;
                }

                // push through synchronizer
                gmskframesync_execute(fs, data_resamp, n);

                // reset counter (again)
                n = 0;
            }

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
    float data_rate = 8.0f * (float)(num_bytes_received) / runtime;
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
    printf("    bytes received      : %6u\n", num_bytes_received);
    printf("    run time            : %f s\n", runtime);
    printf("    data rate           : %12.8f kbps\n", data_rate*1e-3f);
    printf("    spectral efficiency : %12.8f b/s/Hz\n", spectral_efficiency);

    // clean it up
    gmskframesync_destroy(fs);
    resamp_crcf_destroy(resamp);
    resamp2_crcf_destroy(decim);
    timer_destroy(t0);

    return 0;
}

