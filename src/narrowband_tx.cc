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

#include <uhd/usrp/single_usrp.hpp>

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
    printf("  F     : filter type: [rrcos], rkaiser, arkaiser, hM3, gmsk, fexp, fsech, farcsech\n");
}

int main (int argc, char **argv)
{
    // command-line options
    bool verbose = true;

    unsigned long int DAC_RATE = 64e6;
    double min_bandwidth = 40e3f;
    double max_bandwidth = 8000e3f;

    double frequency = 462.0e6;
    double bandwidth = 160e3f;
    float num_seconds = 10.0f;      // run time
    double txgain_dB = -10.0f;      // software tx gain [dB]
    double uhd_txgain = 40.0;       // uhd (hardware) tx gain

    // modulation properties
    modulation_scheme ms = LIQUID_MODEM_QAM;// modulation scheme
    unsigned int bps = 2;                   // modulation depth
    
    // transmit filter properties
    liquid_rnyquist_type ftype = LIQUID_RNYQUIST_RRC;
    unsigned int m = 3;         // matched-filter semi-length
    float beta = 0.3f;          // excess bandwidth factor

    //
    int d;
    while ((d = getopt(argc,argv,"hqvf:b:g:G:t:m:F:")) != EOF) {
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
            liquid_getopt_str2modbps(optarg, &ms, &bps);
            if (ms == LIQUID_MODEM_UNKNOWN) {
                fprintf(stderr,"error: %s, unknown/unsupported mod. scheme: %s\n", argv[0], optarg);
                exit(-1);
            }
            break;
        case 'F':
            if (strcmp(optarg,"rrcos")==0)          ftype = LIQUID_RNYQUIST_RRC;
            else if (strcmp(optarg,"rkaiser")==0)   ftype = LIQUID_RNYQUIST_RKAISER;
            else if (strcmp(optarg,"arkaiser")==0)  ftype = LIQUID_RNYQUIST_ARKAISER;
            else if (strcmp(optarg,"hM3")==0)       ftype = LIQUID_RNYQUIST_hM3;
            else if (strcmp(optarg,"gmsk")==0)      ftype = LIQUID_RNYQUIST_GMSKTX;
            else if (strcmp(optarg,"fexp")==0)      ftype = LIQUID_RNYQUIST_FEXP;
            else if (strcmp(optarg,"fsech")==0)     ftype = LIQUID_RNYQUIST_FSECH;
            else if (strcmp(optarg,"farcsech")==0)  ftype = LIQUID_RNYQUIST_FARCSECH;
            else {
                fprintf(stderr,"error: %s, unknown filter type '%s'\n", argv[0], optarg);
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
    }

    uhd::device_addr_t dev_addr;
    //dev_addr["addr0"] = "192.168.10.2";
    //dev_addr["addr1"] = "192.168.10.3";
    uhd::usrp::single_usrp::sptr usrp = uhd::usrp::single_usrp::make(dev_addr);

    // set properties
    double tx_rate = 2.0*bandwidth;

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

    // force interpolation rate to be in [8,512] (if only this worked...)
    if (interp_rate > 512) interp_rate = 512;
    if (interp_rate < 8)   interp_rate = 8;
    printf("usrp interp rate: %u\n", interp_rate);

    // compute usrp sampling rate
    double usrp_tx_rate = DAC_RATE / (double)interp_rate;
    
    // try to set tx rate
    usrp->set_tx_rate(DAC_RATE / interp_rate);

    // get actual tx rate
    usrp_tx_rate = usrp->get_tx_rate();

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

    // create modem object
    modem mod = modem_create(ms,bps);
    unsigned int M = 1 << modem_get_bps(mod);

    // create matched filter interpolator
    interp_crcf mfinterp = interp_crcf_create_rnyquist(ftype, 2, m, beta, 0);

    // create arbitrary resampler
#if 0
    resamp_crcf resamp = resamp_crcf_create(tx_resamp_rate,7,0.4f,60.0f,64);
    resamp_crcf_setrate(resamp, tx_resamp_rate);
#else
    msresamp_crcf resamp = msresamp_crcf_create(tx_resamp_rate,60.0f);
    printf("arbitrary resampling rate: %f\n", tx_resamp_rate);
#endif

    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/20.0f);

    // buffer lengths
    unsigned int num_symbols = 40;  // number of symbols per buffer
    unsigned int resamp_buffer_len = 2*num_symbols*tx_resamp_rate;
    if (resamp_buffer_len < 2*num_symbols)
        resamp_buffer_len = 2*num_symbols;

    // buffers
    std::complex<float> buffer[num_symbols];
    std::complex<float> buffer_interp[2*num_symbols];
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

        // interpolate by 2
        for (j=0; j<num_symbols; j++)
            interp_crcf_execute(mfinterp, buffer[j], &buffer_interp[2*j]);
        
        // resample
        unsigned int nw;
        unsigned int n=0;
        for (j=0; j<2*num_symbols; j++) {
#if 0
            resamp_crcf_execute(resamp, buffer_interp[j], &buffer_resamp[n], &nw);
#else
            msresamp_crcf_execute(resamp, &buffer_interp[j], 1, &buffer_resamp[n], &nw);
#endif
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
    interp_crcf_destroy(mfinterp);
    timer_destroy(t0);

    return 0;
}

