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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <liquid/liquid.h>

#include <uhd/usrp/single_usrp.hpp>

void usage() {
    printf("narrowband_tx:\n");
    printf("  h     : help\n");
    printf("  f     : center frequency [Hz]\n");
    printf("  b     : bandwidth [Hz]\n");
    printf("  g     : software transmit power gain [dB] (default: -3dB)\n");
    printf("  G     : uhd tx gain [dB] (default: -40dB)\n");
    printf("  t     : run time [seconds]\n");
    printf("  m     : modulation scheme (qpsk default)\n");
    liquid_print_modulation_schemes();
}

int main (int argc, char **argv)
{
    bool verbose = true;

    unsigned long int DAC_RATE = 64e6;
    float min_bandwidth = 0.25f*(DAC_RATE / 256.0);
    float max_bandwidth = 0.25f*(DAC_RATE /   4.0);

    float frequency = 462.0e6;
    float bandwidth = min_bandwidth;
    float num_seconds = 5.0f;
    float txgain_dB = -3.0f;
    double uhd_txgain = -40.0;
    modulation_scheme ms = LIQUID_MODEM_QPSK;   // modulation scheme
    unsigned int bps     = 2;                   // modulation depth (bits/symbol)

    // matched-filter interpolator options
    unsigned int M=4;   // filter samples/symbol
    unsigned int m=3;   // filter delay (symbols)
    float beta=0.7f;    // filter excess bandwidth factor
    int ftype = LIQUID_RNYQUIST_RRC;

    //
    int d;
    while ((d = getopt(argc,argv,"f:b:g:G:t:n:m:p:s:r:c:k:qvuh")) != EOF) {
        switch (d) {
        case 'h':   usage();                        return 0;
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 'g':   txgain_dB = atof(optarg);       break;
        case 'G':   uhd_txgain = atof(optarg);      break;
        case 't':   num_seconds = atof(optarg);     break;
            liquid_getopt_str2modbps(optarg,&ms,&bps);
            break;
        default:
            return 0;
        }
    }

    if (bandwidth > max_bandwidth) {
        fprintf(stderr,"error: maximum bandwidth exceeded (%8.4f MHz)\n", max_bandwidth*1e-6);
        return 1;
    } else if (bandwidth < min_bandwidth) {
        fprintf(stderr,"error: minimum bandwidth exceeded (%8.4f kHz)\n", min_bandwidth*1e-3);
        return 1;
    } else if (ms == LIQUID_MODEM_UNKNOWN) {
        fprintf(stderr,"error: unsupported modulation scheme\n");
        return 0;
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("tx gain     :   %12.8f [dB]\n", txgain_dB);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    // internal options/parameters
    unsigned int block_len = 512;   // number of symbols per block

    uhd::device_addr_t dev_addr;
    //dev_addr["addr0"] = "192.168.10.2";
    //dev_addr["addr1"] = "192.168.10.3";
    uhd::usrp::single_usrp::sptr usrp = uhd::usrp::single_usrp::make(dev_addr);

    // set properties
    double tx_rate = M*bandwidth;

    // NOTE : the sample rate computation MUST be in double precision so
    //        that the UHD can compute its interpolation rate properly
    unsigned int interp_rate = (unsigned int)(DAC_RATE / tx_rate);
    // ensure multiple of 4
    interp_rate = (interp_rate >> 2) << 2;
    interp_rate += 4; // ensure tx_resamp_rate <= 1.0
    // compute usrp sampling rate
    double usrp_tx_rate = DAC_RATE / (double)interp_rate;
    
    // set rate, and get actual rate of hardware
    usrp->set_tx_rate(usrp_tx_rate);
    usrp_tx_rate = usrp->get_tx_rate();

    // compute arbitrary resampling rate
    double tx_resamp_rate = usrp_tx_rate / tx_rate;
    printf("sample rate :   %12.8f kHz = %12.8f / %8.6f (interp %u)\n",
            tx_rate * 1e-3f,
            usrp_tx_rate * 1e-3f,
            tx_resamp_rate,
            interp_rate);

    usrp->set_tx_freq(frequency);
    usrp->set_tx_gain(uhd_txgain);
    // set the IF filter bandwidth
    //usrp->set_tx_bandwidth(2.0f*tx_rate);

    //unsigned int num_blocks = (unsigned int)((4.0f*bandwidth*num_seconds)/(4*frame_len));
    unsigned int num_blocks = (unsigned int)( (M*bandwidth*num_seconds)/(block_len) );

    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/10.0f);
 
    // add arbitrary resampling component
    resamp_crcf resamp = resamp_crcf_create(tx_resamp_rate,7,0.4f,60.0f,64);
    resamp_crcf_setrate(resamp, tx_resamp_rate);

    // create transmitter objects
    modem mod = modem_create(ms, bps);


    // create pulse-shaping interpolator
    interp_crcf mfinterp = interp_crcf_create_rnyquist(ftype,M,m,beta,0);

    // framing buffers
    std::complex<float> block[M*block_len];             // interpolated
    std::complex<float> block_resamp[2*M*frame_len];    // resampler

    // hardware
    uhd::tx_metadata_t md;
    md.start_of_burst = false;  // never SOB when continuous
    md.end_of_burst   = false;  // 
    md.has_time_spec  = false;  // set to false to send immediately
    unsigned int tx_buffer_samples=0;
    std::vector<std::complex<float> > buff(256);

    unsigned int i;
    unsigned int j;
    unsigned int s;
    std::complex<float> sym;
    // start transmitter
    for (i=0; i<num_blocks; i++) {
        for (j=0; j<block_len; j++) {
            // generate random modulation symbol
            s = modem_gen_rand_sym(mod);

            // modulate
            modem_modulate(mod, s, &sym);

            // interpolate using matched filter
            interp_crcf_execute(mfinterp, g*sym, &block[M*j]);
        }

        // run resampler
        unsigned int n=0;
        unsigned int nw;
        for (j=0; j<M*block_len; j++) {
            resamp_crcf_execute(resamp, block[j], &block_resamp[n], &nw);
            n += nw;
        }

        // push samples into buffer
        for (j=0; j<n; j++) {
            buff[tx_buffer_samples++] = block_resamp[j];

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
 
    // send a mini EOB packet
    md.start_of_burst = false;
    md.end_of_burst   = true;
    usrp->get_device()->send("", 0, md,
        uhd::io_type_t::COMPLEX_FLOAT32,
        uhd::device::SEND_MODE_FULL_BUFF
    );

    //finished
    printf("usrp data transfer complete\n");

    // clean allocated objects
    modem_destroy(mod);
    interp_crcf_destroy(mfinterp);
    resamp_crcf_destroy(resamp);
    return 0;
}

