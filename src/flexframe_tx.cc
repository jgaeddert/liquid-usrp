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
    printf("flexframe_tx:\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  b     :   bandwidth [Hz]\n");
    printf("  g     :   transmit power gain [dB] (default -3dB)\n");
    printf("  t     :   run time [seconds]\n");
    printf("  n     :   payload length (bytes)\n");
    printf("  m     :   mod. scheme: <psk>, dpsk, ask, qam, apsk...\n");
    printf("  p     :   mod. depth: <1>,2,...8\n");
    printf("  s     :   packet spacing <0>\n");
    printf("  r     :   ramp up/dn length <64>\n");
    printf("  c     :   fec coding scheme (inner)\n");
    printf("  k     :   fec coding scheme (outer)\n");
    // print all available FEC schemes
    unsigned int i;
    for (i=0; i<LIQUID_FEC_NUM_SCHEMES; i++)
        printf("          [%s] %s\n", fec_scheme_str[i][0], fec_scheme_str[i][1]);
    printf("  q     :   quiet\n");
    printf("  v     :   verbose\n");
    printf("  u,h   :   usage/help\n");
}

int main (int argc, char **argv)
{
    bool verbose = true;

    float min_bandwidth = 0.25f*(64e6 / 256.0);
    float max_bandwidth = 0.25f*(64e6 /   4.0);

    float frequency = 462.0e6;
    float bandwidth = min_bandwidth;
    float num_seconds = 5.0f;
    float txgain_dB = -3.0f;

    unsigned int packet_spacing=0;
    unsigned int payload_len=200;
    fec_scheme fec0 = LIQUID_FEC_NONE;
    fec_scheme fec1 = LIQUID_FEC_HAMMING74;
    modulation_scheme mod_scheme = LIQUID_MODEM_QAM;
    unsigned int mod_depth = 2;
    unsigned int ramp_len = 64;

    //
    int d;
    while ((d = getopt(argc,argv,"f:b:g:t:n:m:p:s:r:c:k:qvuh")) != EOF) {
        switch (d) {
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 'g':   txgain_dB = atof(optarg);       break;
        case 't':   num_seconds = atof(optarg);     break;
        case 'n':   payload_len = atoi(optarg);     break;
        case 'm':
            mod_scheme = liquid_getopt_str2mod(optarg);
            if (mod_scheme == LIQUID_MODEM_UNKNOWN) {
                printf("error: unknown/unsupported mod. scheme: %s\n", optarg);
                mod_scheme = LIQUID_MODEM_UNKNOWN;
            }
            break;
        case 'p':   mod_depth = atoi(optarg);       break;
        case 's':   packet_spacing = atoi(optarg);  break;
        case 'r':   ramp_len = atoi(optarg);        break;
        case 'c':   fec0 = liquid_getopt_str2fec(optarg);         break;
        case 'k':   fec1 = liquid_getopt_str2fec(optarg);         break;
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
    } else if (payload_len > (1<<16)) {
        printf("error: maximum payload length exceeded: %u > %u\n", payload_len, 1<<16);
        return 0;
    } else if (fec0 == LIQUID_FEC_UNKNOWN || fec1 == LIQUID_FEC_UNKNOWN) {
        usage();
        return 0;
    } else if (mod_scheme == LIQUID_MODEM_UNKNOWN) {
        usage();
        return 0;
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("tx gain     :   %12.8f [dB]\n", txgain_dB);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    uhd::device_addr_t dev_addr;
    //dev_addr["addr0"] = "192.168.10.2";
    //dev_addr["addr1"] = "192.168.10.3";
    uhd::usrp::single_usrp::sptr usrp = uhd::usrp::single_usrp::make(dev_addr);

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

    // add arbitrary resampling component
    resamp_crcf resamp = resamp_crcf_create(tx_resamp_rate,7,0.4f,60.0f,64);
    resamp_crcf_setrate(resamp, tx_resamp_rate);

    // create flexframegen object
    flexframegenprops_s fgprops;
    flexframegenprops_init_default(&fgprops);
    fgprops.rampup_len  = ramp_len;
    fgprops.phasing_len = 64;
    fgprops.payload_len = payload_len;
    fgprops.mod_scheme  = mod_scheme;
    fgprops.mod_bps     = mod_depth;
    fgprops.rampdn_len  = ramp_len;

    flexframegen fg = flexframegen_create(&fgprops);
    flexframegen_print(fg);

    // set up the metadta flags
    uhd::tx_metadata_t md;
    md.start_of_burst = false;  // never SOB when continuous
    md.end_of_burst   = false;  // 
    md.has_time_spec  = false;  // set to false to send immediately

    // framing buffers
    unsigned int frame_len = flexframegen_getframelen(fg);
    std::complex<float> frame[frame_len];
#if 0
    std::complex<float> buffer_interp[64];  // matched-filter interpolator (interp by 4)
    std::complex<float> buffer_resamp[128]; // resampler
#else
    std::complex<float> buffer_interp[4*frame_len];  // matched-filter interpolator (interp by 4)
    std::complex<float> buffer_resamp[6*frame_len]; // resampler
    std::vector<std::complex<float> > buff(6*frame_len);
#endif

    printf("frame length        :   %u\n", frame_len);

    unsigned int num_blocks = (unsigned int)((4.0f*bandwidth*num_seconds)/(4*frame_len));

    // create pulse-shaping interpolator
    unsigned int k=4;
    unsigned int m=3;
    float beta=0.7f;
    interp_crcf mfinterp = interp_crcf_create_rnyquist(LIQUID_RNYQUIST_RRC,k,m,beta,0);

    // data buffers
    unsigned char header[9];
    unsigned char payload[payload_len];

    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/10.0f);
 
    unsigned int i, j, pid=0;
    // start transmitter
    for (i=0; i<num_blocks; i++) {
        // generate the frame / transmit silence
        if ((i%(packet_spacing+1))==0) {
            // generate random data
            // TODO : encode using forward error-correction codec
            for (j=0; j<payload_len; j++)
                payload[j] = rand() % 256;
            // write header
            header[0] = (pid >> 8) & 0xff;
            header[1] = (pid     ) & 0xff;
            header[2] = (payload_len >> 8) & 0xff;
            header[3] = (payload_len     ) & 0xff;
            if (verbose)
                printf("packet id: %6u\n", pid);
                //printf("packet id: %6u, packet len : %6u\n", pid, packet_len);
            pid = (pid+1) & 0xffff;

            flexframegen_execute(fg, header, payload, frame);

            // interpolate using matched filter
            for (j=0; j<frame_len; j++)
                interp_crcf_execute(mfinterp, g*frame[j], &buffer_interp[4*j]);
            
        } else {
            // flush interpolator with zeros
            for (j=0; j<frame_len; j++)
                interp_crcf_execute(mfinterp, 0.0f, &buffer_interp[4*j]);
        }

        // run resampler
        unsigned int n=0;
        unsigned int nw;
        for (j=0; j<4*frame_len; j++) {
            resamp_crcf_execute(resamp, buffer_interp[j], &buffer_resamp[n], &nw);
            n += nw;
        }

        //printf(" n = %6u (frame_len : %6u)\n", n, frame_len);
        buff.resize(n);
        for (j=0; j<n; j++) {
            buff[j] = buffer_interp[j];
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
    flexframegen_destroy(fg);
    interp_crcf_destroy(mfinterp);
    resamp_crcf_destroy(resamp);
    return 0;
}

