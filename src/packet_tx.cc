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

    double min_bandwidth = (64e6 / 512.0);
    double max_bandwidth = (64e6 /   4.0);

    double frequency = 462.0e6;
    double bandwidth = min_bandwidth;
    double num_seconds = 5.0f;
    double txgain_dB = -3.0f;

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

    uhd::device_addr_t dev_addr;
    //dev_addr["addr0"] = "192.168.10.2";
    //dev_addr["addr1"] = "192.168.10.3";
    uhd::usrp::single_usrp::sptr usrp = uhd::usrp::single_usrp::make(dev_addr);

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("tx gain     :   %12.8f [dB]\n", txgain_dB);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    // set properties
    double tx_rate = 4.0*bandwidth;
#if 0
    usrp->set_tx_rate(tx_rate);
#else
    // NOTE : the sample rate computation MUST be in double precision so
    //        that the UHD can compute its interpolation rate properly
    unsigned int interp_rate = (unsigned int)(64e6 / tx_rate);
    // ensure multiple of 4
    interp_rate = (interp_rate >> 2) << 2;
    // NOTE : there seems to be a bug where if the interp rate is equal to
    //        240 or 244 we get some weird warning saying that
    //        "The hardware does not support the requested TX sample rate"
    while (interp_rate == 240 || interp_rate == 244)
        interp_rate -= 4;
    // compute usrp sampling rate
    double usrp_tx_rate = 64e6 / (double)interp_rate;
    //usrp_tx_rate = 262295.081967213;
    // compute arbitrary resampling rate
    double tx_resamp_rate = usrp_tx_rate / tx_rate;
    printf("sample rate :   %12.8f kHz = %12.8f * %8.6f (interp %u)\n",
            tx_rate * 1e-3f,
            usrp_tx_rate * 1e-3f,
            1.0 / tx_resamp_rate,
            interp_rate);

    usrp->set_tx_rate(usrp_tx_rate);
#endif
    usrp->set_tx_freq(frequency);
    usrp->set_tx_gain(-40);
    // set the IF filter bandwidth
    //usrp->set_tx_bandwidth(2.0f*tx_rate);

    //assert(tx_resamp_rate <= 1.0);

    // add arbitrary resampling component
    resamp_crcf resamp = resamp_crcf_create(tx_resamp_rate,37,0.4f,60.0f,64);
    resamp_crcf_setrate(resamp, tx_resamp_rate);

    // half-band resampler
    resamp2_crcf interp = resamp2_crcf_create(41,0.0f,40.0f);

    // framegen parameters
    //unsigned int k=2; // samples per symbol
    unsigned int m=3; // delay
    float beta=0.7f;  // excess bandwidth factor

    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/10.0f);
 
    // framing
    unsigned int frame_len = 1280;
    std::complex<float> frame[frame_len];
    std::complex<float> buffer_interp[2*frame_len];
    std::complex<float> buffer_resamp[3*frame_len];
    framegen64 framegen = framegen64_create(m,beta);

    // data buffers
    unsigned char header[24];
    unsigned char payload[64];

    unsigned int j, pid=0;

    // set up the metadta flags
    std::vector<std::complex<float> > buff(2*frame_len);
    uhd::tx_metadata_t md;
    md.start_of_burst = false;  // never SOB when continuous
    md.end_of_burst   = false;  // 
    md.has_time_spec  = false;  // set to false to send immediately

    // compute frame time
    double frame_time = (double)frame_len / bandwidth;
    printf("frame time : %12.8f us\n", frame_time * 1e6f);
    unsigned long int delay = (unsigned long int)(frame_time*1e6f);

    unsigned int i;
    unsigned int num_blocks = (unsigned int)((tx_rate*num_seconds)/(frame_len));
    for (i=0; i<num_blocks; i++) {
        // generate the frame / transmit silence
        //if ((i%(packet_spacing+1))==0) {
        if (1) {
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
            // fill buffer with zeros
            // TODO : only transmit with valid frame data
            for (j=0; j<frame_len; j++)
                frame[j] = 0.0f;
        }

        // resample, apply gain, copy to vector (fill the buffer)
        unsigned int nw;
        unsigned int n = 0;
        for (j=0; j<frame_len; j++) {
            //
            resamp_crcf_execute(resamp, frame[j], &buffer_resamp[n], &nw);
            n += nw;
        }
        
        // interpolate by 2
        for (j=0; j<n; j++) {
            //
            resamp2_crcf_interp_execute(interp, buffer_resamp[j], &buffer_interp[2*j]);
        }
        
        //printf(" n = %6u (frame_len : %6u)\n", n, frame_len);
        buff.resize(2*n);
        for (j=0; j<2*n; j++) {
            buff[j] = g*buffer_interp[j];
        }

        //send the entire contents of the buffer
        usrp->get_device()->send(
            &buff.front(), buff.size(), md,
            uhd::io_type_t::COMPLEX_FLOAT32,
            uhd::device::SEND_MODE_FULL_BUFF
        );

    }
 
    // send a mini EOB packet
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
    resamp_crcf_destroy(resamp);
    resamp2_crcf_destroy(interp);
    return 0;
}

