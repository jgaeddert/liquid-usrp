/*
 * Copyright (c) 2011, 2013 Joseph Gaeddert
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

#include <uhd/usrp/multi_usrp.hpp>

#include "timer.h"

void usage() {
    printf("narrowband_tx [OPTION]\n");
    printf("transmit narrowband signal (random data)\n");
    printf("\n");
    printf("  h     : usage/help\n");
    printf("  q/v   : quiet/verbose\n");
    printf("  f     : center frequency [Hz]\n");
    printf("  b     : symbol rate [Hz] (40kHz min, 8MHz max), default: 160kHz\n");
    printf("  g     : software tx gain [dB] (default: -10dB)\n");
    printf("  G     : uhd tx gain [dB] (default: 40dB)\n");
    printf("  t     : execute time [s], default: 10\n");
    printf("  m     : modulation scheme (qpsk default)\n");
    printf("  F     : matched filter type: [rrcos], rkaiser, arkaiser, hM3, gmsk, fexp, fsech, farcsech\n");
    printf("  K     : matched filter samples/symbol,  default: 2\n");
    printf("  M     : matched filter semi-length,     default: 9\n");
    printf("  B     : matchedfilter excess bandwidth, default: 0.2\n");
}

int main (int argc, char **argv)
{
    // command-line options
    bool verbose = true;

    double frequency = 462.0e6;
    double bandwidth = 160e3f;
    float num_seconds = 10.0f;      // run time
    double txgain_dB = -10.0f;      // software tx gain [dB]
    double uhd_txgain = 40.0;       // uhd (hardware) tx gain

    // modulation properties
    modulation_scheme ms = LIQUID_MODEM_QPSK;// modulation scheme
    
    // transmit filter properties
    liquid_firfilt_type ftype = LIQUID_FIRFILT_RRC;
    unsigned int k = 2;         // matched-filter samples/symbol
    unsigned int m = 9;         // matched-filter semi-length
    float beta     = 0.2f;      // excess bandwidth factor

    //
    int d;
    while ((d = getopt(argc,argv,"hqvf:b:g:G:t:m:F:K:M:B:")) != EOF) {
        switch (d) {
        case 'h':   usage();                        return 0;
        case 'q':   verbose = false;                break;
        case 'v':   verbose = true;                 break;
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 'g':   txgain_dB = atof(optarg);       break;
        case 'G':   uhd_txgain = atof(optarg);      break;
        case 't':   num_seconds = atof(optarg);     break;
        case 'm':
            ms = liquid_getopt_str2mod(optarg);
            if (ms == LIQUID_MODEM_UNKNOWN) {
                fprintf(stderr,"error: %s, unknown/unsupported mod. scheme: %s\n", argv[0], optarg);
                exit(-1);
            }
            break;
        case 'F':
            if (strcmp(optarg,"rrcos")==0)          ftype = LIQUID_FIRFILT_RRC;
            else if (strcmp(optarg,"rkaiser")==0)   ftype = LIQUID_FIRFILT_RKAISER;
            else if (strcmp(optarg,"arkaiser")==0)  ftype = LIQUID_FIRFILT_ARKAISER;
            else if (strcmp(optarg,"hM3")==0)       ftype = LIQUID_FIRFILT_hM3;
            else if (strcmp(optarg,"gmsk")==0)      ftype = LIQUID_FIRFILT_GMSKTX;
            else if (strcmp(optarg,"fexp")==0)      ftype = LIQUID_FIRFILT_FEXP;
            else if (strcmp(optarg,"fsech")==0)     ftype = LIQUID_FIRFILT_FSECH;
            else if (strcmp(optarg,"farcsech")==0)  ftype = LIQUID_FIRFILT_FARCSECH;
            else {
                fprintf(stderr,"error: %s, unknown filter type '%s'\n", argv[0], optarg);
                exit(1);
            }
            break;
        case 'K':   k    = atoi(optarg);    break;
        case 'M':   m    = atoi(optarg);    break;
        case 'B':   beta = atof(optarg);    break;
        default:
            usage();
            return 0;
        }
    }

    if (k < 2) {
        fprintf(stderr,"error: %s, filter samples/symbol must be at least 2\n", argv[0]);
        exit(1);
    } else if (m < 1) {
        fprintf(stderr,"error: %s, filter semi-length must be at least 1\n", argv[0]);
        exit(1);
    } else if (beta <= 0.0f || beta > 1.0f) {
        fprintf(stderr,"error: %s, filter excess bandwidth must be in (0, 1]\n", argv[0]);
        exit(1);
    }

    uhd::device_addr_t dev_addr;
    //dev_addr["addr0"] = "192.168.10.2";
    //dev_addr["addr1"] = "192.168.10.3";
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);

    // try to set tx rate (oversampled to compensate for CIC filter)
    usrp->set_tx_rate(2.0f * k * bandwidth);

    // get actual tx rate
    double usrp_tx_rate = usrp->get_tx_rate();

    // compute arbitrary resampling rate (make up the difference in software)
    double tx_resamp_rate = usrp_tx_rate / (k * bandwidth);

    usrp->set_tx_freq(frequency);
    usrp->set_tx_gain(uhd_txgain);

    printf("frequency       :   %10.4f [MHz]\n", frequency*1e-6f);
    printf("bandwidth       :   %10.4f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity       :   %s\n", (verbose?"enabled":"disabled"));
    printf("usrp sample rate:   %.4f kHz = %.4f kHz * %u samp/sym * %.4f resamp rate\n",
            usrp_tx_rate * 1e-3f,
            bandwidth    * 1e-3f,
            k,
            tx_resamp_rate);

    // set the IF filter bandwidth
    //usrp->set_tx_bandwidth(2.0f*tx_rate);

    // create modem object
    modem mod = modem_create(ms);
    unsigned int M = 1 << modem_get_bps(mod);

    // create matched filter interpolator
    firinterp_crcf mfinterp = firinterp_crcf_create_rnyquist(ftype, k, m, beta, 0);

    // create arbitrary resampler
    msresamp_crcf resamp = msresamp_crcf_create(tx_resamp_rate,60.0f);

    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/20.0f);

    // buffer lengths
    unsigned int num_symbols = 40;  // number of symbols per buffer
    unsigned int resamp_buffer_len = 64 + ceilf(2.2*k*num_symbols*tx_resamp_rate);
    if (resamp_buffer_len < k*num_symbols)
        resamp_buffer_len = k*num_symbols;

    // buffers
    std::complex<float> buffer[num_symbols];
    std::complex<float> buffer_interp[k*num_symbols];
    std::complex<float> buffer_resamp[resamp_buffer_len];

    // set up the metadata flags
    std::vector<std::complex<float> > buff(256);
    unsigned int tx_buffer_samples=0;
    uhd::tx_metadata_t md;
    md.start_of_burst = false;  // never SOB when continuous
    md.end_of_burst   = false;  // 
    md.has_time_spec  = false;  // set to false to send immediately

    // run conditions
    int continue_running = 1;
    timer t0 = timer_create();
    timer_tic(t0);
    
    unsigned int j;
    while (continue_running) {
        // generate random modem symbols
        for (j=0; j<num_symbols; j++)
            modem_modulate(mod, rand()%M, &buffer[j]);

        // interpolate by k
        for (j=0; j<num_symbols; j++)
            firinterp_crcf_execute(mfinterp, buffer[j], &buffer_interp[k*j]);
        
        // resample
        unsigned int nw;
        unsigned int n=0;
        for (j=0; j<k*num_symbols; j++) {
            msresamp_crcf_execute(resamp, &buffer_interp[j], 1, &buffer_resamp[n], &nw);
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

        // check runtime
        if (timer_toc(t0) >= num_seconds)
            continue_running = 0;
    }
 
    // send a mini EOB packet
    md.start_of_burst = false;
    md.end_of_burst   = true;
    usrp->get_device()->send("", 0, md,
        uhd::io_type_t::COMPLEX_FLOAT32,
        uhd::device::SEND_MODE_FULL_BUFF);

    // sleep for a small amount of time to allow USRP buffers
    // to flush
    usleep(100000);

    //finished
    printf("usrp data transfer complete\n");

    // clean it up
#if 0
    resamp_crcf_destroy(resamp);
#else
    msresamp_crcf_destroy(resamp);
#endif
    modem_destroy(mod);
    firinterp_crcf_destroy(mfinterp);
    timer_destroy(t0);

    return 0;
}

