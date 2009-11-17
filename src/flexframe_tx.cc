/*
 * Copyright (c) 2007, 2009 Joseph Gaeddert
 * Copyright (c) 2007, 2009 Virginia Polytechnic Institute & State University
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

#include <math.h>
#include <iostream>
#include <complex>
#include <getopt.h>
#include <liquid/liquid.h>

#include "usrp_io.h"
 
#define USRP_CHANNEL        (0)

void usage() {
    printf("flexframe_tx:\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  b     :   bandwidth [Hz]\n");
    printf("  t     :   run time [seconds]\n");
    printf("  n     :   payload length (bytes)\n");
    printf("  m     :   mod. scheme: <psk>, dpsk, ask, qam,...\n");
    printf("  p     :   mod. depth: <1>,2,...8\n");
    printf("  c,k   :   fec coding scheme: c (inner), k (outer)\n");
    printf("                <none>, h74, r3,\n");
    printf("                v27, v29, v39, v615,\n");
    printf("                v27p23, v27p34, v27p45, v27p56, v27p67, v27p78,\n");
    printf("                v29p23, v29p34, v29p45, v29p56, v29p67, v29p78,\n");
    printf("  q     :   quiet\n");
    printf("  v     :   verbose\n");
    printf("  u,h   :   usage/help\n");
}

// return scheme from string
fec_scheme str2fec(const char * _str);

int main (int argc, char **argv)
{
    bool verbose = true;

    float min_bandwidth = (32e6 / 512.0);
    float max_bandwidth = (32e6 /   4.0);

    float frequency = 462.0e6;
    float bandwidth = min_bandwidth;
    float num_seconds = 5.0f;

    unsigned int packet_spacing=0;
    unsigned int payload_len=200;
    fec_scheme fec0 = FEC_NONE;
    fec_scheme fec1 = FEC_HAMMING74;
    modulation_scheme mod_scheme = MOD_QAM;
    unsigned int mod_depth = 2;

    //
    int d;
    while ((d = getopt(argc,argv,"f:b:t:n:m:p:c:k:qvuh")) != EOF) {
        switch (d) {
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 't':   num_seconds = atof(optarg);     break;
        case 'n':   payload_len = atoi(optarg);     break;
        case 'm':
            if (strcmp(optarg,"psk")==0) {
                mod_scheme = MOD_PSK;
            } else if (strcmp(optarg, "dpsk")==0) {
                mod_scheme = MOD_DPSK;
            } else if (strcmp(optarg, "ask")==0) {
                mod_scheme = MOD_ASK;
            } else if (strcmp(optarg, "qam")==0) {
                mod_scheme = MOD_QAM;
            } else {
                printf("error: unknown mod. scheme: %s\n", optarg);
                mod_scheme = MOD_UNKNOWN;
            }
            break;
        case 'p':   mod_depth = atoi(optarg);       break;
        //case 'p':   packet_spacing = atoi(optarg);  break;
        case 'c':   fec0 = str2fec(optarg);         break;
        case 'k':   fec1 = str2fec(optarg);         break;
        case 'q':   verbose = false;                break;
        case 'v':   verbose = true;                 break;
        case 'u':
        case 'h':
        default:
            usage();
            return 0;
        }
    }

    // compute interpolation rate
    unsigned int interp_rate = (unsigned int)(32e6 / bandwidth);
    
    // ensure multiple of 4
    interp_rate = (interp_rate >> 2) << 2;

    // update actual bandwidth
    bandwidth = 32e6f / (float)(interp_rate);

    if (bandwidth > max_bandwidth) {
        printf("error: maximum bandwidth exceeded (%8.4f MHz)\n", max_bandwidth*1e-6);
        return 0;
    } else if (bandwidth < min_bandwidth) {
        printf("error: minimum bandwidth exceeded (%8.4f kHz)\n", min_bandwidth*1e-3);
        return 0;
    } else if (payload_len > (1<<16)) {
        printf("error: maximum payload length exceeded: %u > %u\n", payload_len, 1<<16);
        return 0;
    } else if (fec0 == FEC_UNKNOWN || fec1 == FEC_UNKNOWN) {
        usage();
        return 0;
    } else if (mod_scheme == MOD_UNKNOWN) {
        usage();
        return 0;
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    // create usrp_io object and set properties
    usrp_io * uio = new usrp_io();
    uio->set_tx_freq(USRP_CHANNEL, frequency);
    uio->set_tx_interp(interp_rate);
    uio->enable_auto_tx(USRP_CHANNEL);

    // retrieve tx port
    gport port_tx = uio->get_tx_port(USRP_CHANNEL);

    // packetizer
    packetizer p = packetizer_create(payload_len,fec0,fec1);
    unsigned int packet_len = packetizer_get_packet_length(payload_len,fec0,fec1);
    packetizer_print(p);

    // create flexframegen object
    flexframegenprops_s fgprops;
    fgprops.rampup_len = 64;
    fgprops.phasing_len = 64;
    fgprops.payload_len = packet_len;
    fgprops.mod_scheme = mod_scheme;
    fgprops.mod_bps = mod_depth;
    fgprops.rampdn_len = 64;
    flexframegen fg = flexframegen_create(&fgprops);
    flexframegen_print(fg);


    // framing buffers
    unsigned int frame_len = flexframegen_getframelen(fg);
    std::complex<float> frame[frame_len];
    std::complex<float> mfbuffer[2*frame_len];

    printf("frame length        :   %u\n", frame_len);

    unsigned int num_blocks = (unsigned int)((4.0f*bandwidth*num_seconds)/(4*frame_len));

    // create pulse-shaping interpolator
    unsigned int m=3;
    float beta=0.7f;
    unsigned int h_len = 2*2*m + 1;
    float h[h_len];
    design_rrc_filter(2,m,beta,0,h);
    interp_crcf mfinterp = interp_crcf_create(2,h,h_len);

    // create half-band interpolator
    resamp2_crcf interpolator = resamp2_crcf_create(37,0.0f,60.0f);

    // data buffers
    unsigned char header[8];
    unsigned char payload[payload_len];
    unsigned char packet[packet_len];

    // start usrp data transfer
    uio->start_tx(USRP_CHANNEL);

    std::complex<float> * data_tx;
 
    unsigned int i, j, n, pid=0;
    // Do USRP Samples Reading 
    for (i=0; i<num_blocks; i++) {
        // generate the frame / transmit silence
        if ((i%(packet_spacing+1))==0) {
            // generate random data
            // TODO : encode using forward error-correction codec
            for (j=0; j<payload_len; j++)
                payload[j] = rand() % 256;
            // assemble packet
            packetizer_encode(p,payload,packet);
            // write header
            header[0] = (pid >> 8) & 0xff;
            header[1] = (pid     ) & 0xff;
            header[2] = (payload_len >> 8) & 0xff;
            header[3] = (payload_len     ) & 0xff;
            header[4] = (unsigned char)(fec0);
            header[5] = (unsigned char)(fec1);
            if (verbose)
                printf("packet id: %6u\n", pid);
                //printf("packet id: %6u, packet len : %6u\n", pid, packet_len);
            /*
            for (j=0; j<payload_len; j++)
                printf("%.2x ",payload[j]);
            printf("\n");
            for (j=0; j<packet_len; j++)
                printf("%.2x ",packet[j]);
            printf("\n");
            */
            pid = (pid+1)%(1<<16);

            flexframegen_execute(fg, header, packet, frame);

            // interpolate using matched filter
            for (j=0; j<frame_len; j++) {
                std::complex<float> x = j<frame_len ? frame[j] : 0.0f;
                interp_crcf_execute(mfinterp, x, &mfbuffer[2*j]);
            }

        } else {
            // flush interpolator with zeros
            for (j=0; j<frame_len; j++)
                interp_crcf_execute(mfinterp, 0.0f, &mfbuffer[2*j]);
        }

        // send data to usrp_io in blocks
        unsigned int num_samples_remaining = 2*frame_len;
        n = 0;
        unsigned int p;

        while (num_samples_remaining > 0) {
            p = num_samples_remaining > 256 ? 256 : num_samples_remaining;

            data_tx = (std::complex<float>*) gport_producer_lock(port_tx,2*p);

            // interpolate using half-band interpolator
            for (j=0; j<p; j++)
                resamp2_crcf_interp_execute(interpolator, mfbuffer[n+j], &data_tx[2*j]);
            n+=p;
            num_samples_remaining -= p;

            gport_producer_unlock(port_tx,2*p);
        }
 
    }
 
 
    uio->stop_tx(USRP_CHANNEL);  // Stop data transfer

    // clean it up
    packetizer_destroy(p);
    flexframegen_destroy(fg);
    resamp2_crcf_destroy(interpolator);
    interp_crcf_destroy(mfinterp);
    delete uio;
    return 0;
}

fec_scheme str2fec(const char * _str)
{
    if (strcmp(_str,"none")==0) {
        return FEC_NONE;
    } else if (strcmp(_str, "v27")==0) {
        return FEC_CONV_V27;
    } else if (strcmp(_str, "v29")==0) {
        return FEC_CONV_V29;
    } else if (strcmp(_str, "v39")==0) {
        return FEC_CONV_V39;
    } else if (strcmp(_str, "v615")==0) {
        return FEC_CONV_V615;
    } else if (strcmp(_str, "v27p23")==0) {
        return FEC_CONV_V27P23;
    } else if (strcmp(_str, "v27p34")==0) {
        return FEC_CONV_V27P34;
    } else if (strcmp(_str, "v27p45")==0) {
        return FEC_CONV_V27P45;
    } else if (strcmp(_str, "v27p56")==0) {
        return FEC_CONV_V27P56;
    } else if (strcmp(_str, "v27p67")==0) {
        return FEC_CONV_V27P67;
    } else if (strcmp(_str, "v27p78")==0) {
        return FEC_CONV_V27P78;
    } else if (strcmp(_str, "v29p23")==0) {
        return FEC_CONV_V29P23;
    } else if (strcmp(_str, "v29p34")==0) {
        return FEC_CONV_V29P34;
    } else if (strcmp(_str, "v29p45")==0) {
        return FEC_CONV_V29P45;
    } else if (strcmp(_str, "v29p56")==0) {
        return FEC_CONV_V29P56;
    } else if (strcmp(_str, "v29p67")==0) {
        return FEC_CONV_V29P67;
    } else if (strcmp(_str, "v29p78")==0) {
        return FEC_CONV_V29P78;
    } else if (strcmp(_str, "r3")==0) {
        return FEC_REP3;
    } else if (strcmp(_str, "h74")==0) {
        return FEC_HAMMING74;
    } else {
        printf("unknown/unsupported fec scheme : %s\n", _str);
    }
    return FEC_UNKNOWN;
}


