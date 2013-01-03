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
 
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <complex>
#include <getopt.h>

#include <liquid/liquid.h>
#include <liquid/liquid-wlan.h>

#include <uhd/usrp/multi_usrp.hpp>

void usage() {
    printf("ofdmflexframe_tx [OPTION]\n");
    printf("transmit OFDM packets\n");
    printf("\n");
    printf("  u,h   : usage/help\n");
    printf("  q/v   : quiet/verbose\n");
    printf("  f     : center frequency [Hz]\n");
    printf("  b     : bandwidth [Hz] (62.5kHz min, 8MHz max)\n");
    printf("  g     : software tx gain [dB] (default: -6dB)\n");
    printf("  G     : uhd tx gain [dB] (default: 40dB)\n");
    printf("  n     : number of data bytes, [1,4095]\n");
    printf("  r     : rate {6,9,12,18,24,36,48,54} M bits/s\n");
#if 0
    printf("  N     : number of frames, default: 1000\n");
    printf("  P     : payload length [bytes], default: 256\n");
#endif
}

int main (int argc, char **argv)
{
    // command-line options
    bool verbose = true;

    unsigned long int DAC_RATE = 64e6;
    double min_bandwidth = 0.25*(DAC_RATE / 512.0);
    double max_bandwidth = 0.25*(DAC_RATE /   4.0);

    double frequency = 462.0e6;
    double bandwidth = 200e3f;
    unsigned int num_frames = 1000;     // number of frames to transmit
    double txgain_dB = -15.0f;          // software tx gain [dB]
    double uhd_txgain = 40.0;           // uhd (hardware) tx gain

    // WLAN properties
    int rate = WLANFRAME_RATE_18;       // WLAN frame data rate
    unsigned int payload_len = 200;     // paylaod length

    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:g:G:n:r:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose = false;                break;
        case 'v':   verbose = true;                 break;
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 'g':   txgain_dB = atof(optarg);       break;
        case 'G':   uhd_txgain = atof(optarg);      break;
        case 'n':   payload_len = atoi(optarg);     break;
        case 'r':
            switch ( atoi(optarg) ) {
            case 6:  rate = WLANFRAME_RATE_6;       break;
            case 9:  rate = WLANFRAME_RATE_9;       break;
            case 12: rate = WLANFRAME_RATE_12;      break;
            case 18: rate = WLANFRAME_RATE_18;      break;
            case 24: rate = WLANFRAME_RATE_24;      break;
            case 36: rate = WLANFRAME_RATE_36;      break;
            case 48: rate = WLANFRAME_RATE_48;      break;
            case 54: rate = WLANFRAME_RATE_54;      break;
            default:
                fprintf(stderr,"error: %s, invalid rate '%s'\n", argv[0], optarg);
                exit(1);
            }
            break;
        default:
            usage();
            return 0;
        }
    }

    if (bandwidth > max_bandwidth) {
        fprintf(stderr,"error: %s, maximum bandwidth exceeded (%8.4f MHz)\n", argv[0], max_bandwidth*1e-6);
        return 0;
    } else if (bandwidth < min_bandwidth) {
        fprintf(stderr,"error: %s, minimum bandwidth exceeded (%8.4f kHz)\n", argv[0], min_bandwidth*1e-3);
        exit(1);
    } else if (payload_len < 1 || payload_len > 4095) {
        fprintf(stderr,"error: %s, payload length must be in [1,4095]\n", argv[0]);
        exit(1);
    }

    uhd::device_addr_t dev_addr;
    //dev_addr["addr0"] = "192.168.10.2";
    //dev_addr["addr1"] = "192.168.10.3";
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);

    // set properties
    double tx_rate = 4.0*bandwidth;

    // NOTE : the sample rate computation MUST be in double precision so
    //        that the UHD can compute its interpolation rate properly
    unsigned int interp_rate = (unsigned int)(DAC_RATE / tx_rate);
    // ensure multiple of 4
    interp_rate = (interp_rate >> 2) << 2;
    // NOTE : there seems to be a bug where if the interp rate is equal to
    //        240 or 244 we get some weird warning saying that
    //        "The hardware does not support the requested TX sample rate"
    while (interp_rate == 240 || interp_rate == 244)
        interp_rate -= 4;
    // compute usrp sampling rate
    double usrp_tx_rate = DAC_RATE / (double)interp_rate;
    
    // try to set tx rate
    usrp->set_tx_rate(DAC_RATE / interp_rate);

    // get actual tx rate
    usrp_tx_rate = usrp->get_tx_rate();

    //usrp_tx_rate = 262295.081967213;
    // compute arbitrary resampling rate
    double tx_resamp_rate = usrp_tx_rate / tx_rate;

    usrp->set_tx_freq(frequency);
    usrp->set_tx_gain(uhd_txgain);

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    printf("sample rate :   %12.8f kHz = %12.8f * %8.6f (interp %u)\n",
            tx_rate * 1e-3f,
            usrp_tx_rate * 1e-3f,
            1.0 / tx_resamp_rate,
            interp_rate);

    // set the IF filter bandwidth
    //usrp->set_tx_bandwidth(2.0f*tx_rate);


    // add arbitrary resampling component
    resamp_crcf resamp = resamp_crcf_create(tx_resamp_rate,7,0.4f,60.0f,64);
    resamp_crcf_setrate(resamp, tx_resamp_rate);

    // half-band resampler
    resamp2_crcf interp = resamp2_crcf_create(7,0.0f,40.0f);

    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/20.0f);

    // data arrays/objects
    struct wlan_txvector_s txvector;
    txvector.LENGTH      = payload_len;
    txvector.DATARATE    = rate;
    txvector.SERVICE     = 0;
    txvector.TXPWR_LEVEL = 0;

    unsigned char payload[payload_len];
    
    // create frame generator
    wlanframegen fg = wlanframegen_create();

    // arrays
    std::complex<float> buffer[80];    // output time series
    std::complex<float> buffer_interp[2*80];
    std::complex<float> buffer_resamp[3*80];

    // set up the metadta flags
    std::vector<std::complex<float> > buff(256);
    unsigned int tx_buffer_samples;
    uhd::tx_metadata_t md;
    md.start_of_burst = false;  // never SOB when continuous
    md.end_of_burst   = false;  // 
    md.has_time_spec  = false;  // set to false to send immediately

    unsigned int j;
    unsigned int pid;
    tx_buffer_samples=0;
    for (pid=0; pid<num_frames; pid++) {
        // reset frame generator (resets pilot generator, etc.)
        wlanframegen_reset(fg);

        if (verbose)
            printf("tx packet id: %6u\n", pid);
        
        // initialize payload
        for (j=0; j<payload_len; j++)
            payload[j] = rand() & 0xff;

        // assemble frame
        wlanframegen_assemble(fg, payload, txvector);

        // generate frame
        int last_symbol=0;
        unsigned int zero_pad=1;
        while (!last_symbol) {
#if 0
            if (!last_symbol) {
                // generate symbol
                last_symbol = wlanframegen_writesymbol(fg, buffer);
            } else {
                zero_pad--;
                for (j=0; j<80; j++)
                    buffer[j] = 0.0f;
            }
#else
            // generate symbol
            last_symbol = wlanframegen_writesymbol(fg, buffer);
#endif

            // interpolate by 2
            for (j=0; j<80; j++)
                resamp2_crcf_interp_execute(interp, buffer[j], &buffer_interp[2*j]);
            
            // resample
            unsigned int nw;
            unsigned int n=0;
            for (j=0; j<2*80; j++) {
                resamp_crcf_execute(resamp, buffer_interp[j], &buffer_resamp[n], &nw);
                n += nw;
            }

            // push samples into buffer
            for (j=0; j<n; j++) {
                buff[tx_buffer_samples++] = g*buffer_resamp[j];

                if (tx_buffer_samples==256) {
                    // reset counter
                    tx_buffer_samples=0;

                    //send the entire contents of the buffer
                    usrp->get_device()->send(
                        &buff.front(), buff.size(), md,
                        uhd::io_type_t::COMPLEX_FLOAT32,
                        uhd::device::SEND_MODE_FULL_BUFF
                    );
                }
            }
        }


#if 0
        // pad remaining samples with zeros
        for (j=num_samples; j<frame_len; j++)
            frame[j] = 0.0f;
#endif
    }
 
    // send a mini EOB packet
    md.start_of_burst = false;
    md.end_of_burst   = true;
    usrp->get_device()->send("", 0, md,
        uhd::io_type_t::COMPLEX_FLOAT32,
        uhd::device::SEND_MODE_FULL_BUFF
    );

    // sleep for a small amount of time to allow USRP buffers
    // to flush
    usleep(100000);

    //finished
    printf("usrp data transfer complete\n");

    // clean it up
    wlanframegen_destroy(fg);
    resamp2_crcf_destroy(interp);
    resamp_crcf_destroy(resamp);

    return 0;
}

