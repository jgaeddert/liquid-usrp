/*
 * Copyright (c) 2007, 2008, 2009, 2010 Joseph Gaeddert
 * Copyright (c) 2007, 2008, 2009, 2010 Virginia Polytechnic
 *                                      Institute & State University
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
#include <liquid/liquid.h>

#include <uhd/usrp/single_usrp.hpp>
 
void usage() {
    printf("packet_tx:\n");
    printf("  f     :   center frequency [Hz] (default: 462 MHz)\n");
    printf("  b     :   bandwidth [Hz], [62.5kHz, 8MHz] (default: 62.5kHz)\n");
    printf("  p     :   packet spacing (default: 1)\n");
    printf("  g     :   transmit power gain [dB] (default: -3dB)\n");
    printf("  t     :   run time [seconds] (default: 5.0)\n");
    printf("  q     :   quiet\n");
    printf("  v     :   verbose\n");
    printf("  u,h   :   usage/help\n");
}

int main (int argc, char **argv)
{
    // command-line options
    bool verbose = true;

    float min_bandwidth = (64e6 / 512.0);
    float max_bandwidth = (64e6 /   4.0);

    float frequency = 462.0e6;
    float bandwidth = min_bandwidth;
    float num_seconds = 5.0f;
    float txgain_dB = -3.0f;

    unsigned int packet_spacing=1;

    //
    int d;
    while ((d = getopt(argc,argv,"f:b:p:g:t:qvuh")) != EOF) {
        switch (d) {
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 'p':   packet_spacing = atoi(optarg);  break;
        case 'g':   txgain_dB = atof(optarg);       break;
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

    if (bandwidth > max_bandwidth) {
        printf("error: maximum bandwidth exceeded (%8.4f MHz)\n", max_bandwidth*1e-6);
        return 0;
    } else if (bandwidth < min_bandwidth) {
        printf("error: minimum bandwidth exceeded (%8.4f kHz)\n", min_bandwidth*1e-3);
        return 0;
    }

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);

    stream_cmd.stream_now = true;

    uhd::device_addr_t dev_addr;
    //dev_addr["addr0"] = "192.168.10.2";
    //dev_addr["addr1"] = "192.168.10.3";
    uhd::usrp::single_usrp::sptr usrp = uhd::usrp::single_usrp::make(dev_addr);

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("tx gain     :   %12.8f [dB]\n", txgain_dB);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    // set properties
    float tx_rate = 2.0f*bandwidth;
#if 1
    usrp->set_tx_rate(tx_rate);
#else
    // NOTE : the sample rate computation MUST be in double precision so
    //        that the UHD can compute its decimation rate properly
    unsigned int decim_rate = (unsigned int)(64e6 / tx_rate);
    // ensure multiple of 2
    decim_rate = (decim_rate >> 1) << 1;
    // compute usrp sampling rate
    double usrp_tx_rate = 64e6 / (float)decim_rate;
    // compute arbitrary resampling rate
    double tx_resamp_rate = tx_rate / usrp_tx_rate;
    printf("sample rate :   %12.8f kHz = %12.8f * %8.6f (decim %u)\n",
            tx_rate * 1e-3f,
            usrp_tx_rate * 1e-3f,
            tx_resamp_rate,
            decim_rate);
    usrp->set_tx_rate(usrp_tx_rate);
#endif
    usrp->set_tx_freq(frequency);
    usrp->set_tx_gain(-20);
    // set the IF filter bandwidth
    //usrp->set_tx_bandwidth(2.0f*tx_rate);

    // framegen parameters
    //unsigned int k=2; // samples per symbol
    unsigned int m=3; // delay
    float beta=0.7f;  // excess bandwidth factor

    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/10.0f);
 
    // framing
    unsigned int frame_len = 1280;
    std::complex<float> frame[frame_len];
    framegen64 framegen = framegen64_create(m,beta);

    // data buffers
    unsigned char header[24];
    unsigned char payload[64];

    unsigned int j, pid=0;

    // set up the metadta flags and start usrp data transfer
    std::vector<std::complex<float> > buff(1280);
    uhd::tx_metadata_t md;
    md.start_of_burst = false; //never SOB when continuous
    md.end_of_burst   = false;

    unsigned int i;
    unsigned int num_blocks = (unsigned int)((tx_rate*num_seconds)/(frame_len));
    for (i=0; i<num_blocks; i++) {
        // generate the frame / transmit silence
        if ((i%(packet_spacing+1))==0) {
            // generate random data
            for (j=0; j<24; j++)    header[j]  = rand() % 256;
            for (j=0; j<64; j++)    payload[j] = rand() % 256;
            header[0] = (pid >> 8) & 0x00ff;
            header[1] = (pid     ) & 0x00ff;
            if (verbose)
                printf("packet id: %u\n", pid);
            pid = (pid+1) & 0xffff;

            framegen64_execute(framegen, header, payload, frame);
        } else {
            // clear frame buffer
            for (j=0; j<frame_len; j++)
                frame[j] = 0.0f;
        }

        // apply gain, copy to vector (fill the buffer)
        for (j=0; j<frame_len; j++)
            buff[j] = g*frame[j];

        //send the entire contents of the buffer
        usrp->get_device()->send(
            &buff.front(), buff.size(), md,
            uhd::io_type_t::COMPLEX_FLOAT32,
            uhd::device::SEND_MODE_FULL_BUFF
        );
    }
 
    //send a mini EOB packet
    md.start_of_burst = false;
    md.end_of_burst   = true;
    usrp->get_device()->send("", 0, md,
        uhd::io_type_t::COMPLEX_FLOAT32,
        uhd::device::SEND_MODE_FULL_BUFF
    );

    //finished
    printf("usrp data transfer complete\n");

    // clean it up
    framegen64_destroy(framegen);
    return 0;
}

