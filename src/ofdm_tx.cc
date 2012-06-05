/*
 * Copyright (c) 2011, 2012 Joseph Gaeddert
 * Copyright (c) 2011, 2012 Virginia Polytechnic Institute & State University
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

void usage() {
    printf("ofdm_tx [OPTION]\n");
    printf("transmit OFDM packets\n");
    printf("\n");
    printf("  u,h   : usage/help\n");
    printf("  q/v   : quiet/verbose\n");
    printf("  f     : center frequency [Hz]\n");
    printf("  b     : bandwidth [Hz] (62.5kHz min, 8MHz max)\n");
    printf("  g     : software tx gain [dB] (default: -6dB)\n");
    printf("  G     : uhd tx gain [dB] (default: 40dB)\n");
}

int main (int argc, char **argv)
{
    // command-line options
    bool verbose = true;

    unsigned long int DAC_RATE = 64e6;
    double min_bandwidth = 0.25*(DAC_RATE / 512.0);
    double max_bandwidth = 0.25*(DAC_RATE /   4.0);

    double frequency = 462.0e6;
    double bandwidth = 80e3f;
    unsigned int num_frames = 1000;     // number of frames to transmit
    unsigned int num_data_symbols = 50; // number of data symbols per frame
    double txgain_dB = -12.0f;          // software tx gain [dB]
    double uhd_txgain = 40.0;           // uhd (hardware) tx gain

    // ofdm properties
    unsigned int M = 64;                // number of subcarriers
    unsigned int cp_len = 8;            // cyclic prefix length
    unsigned int taper_len = 0;         // cyclic prefix length

    modulation_scheme ms = LIQUID_MODEM_QPSK;// modulation scheme
    
    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:g:G:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose     = false;            break;
        case 'v':   verbose     = true;             break;
        case 'f':   frequency   = atof(optarg);     break;
        case 'b':   bandwidth   = atof(optarg);     break;
        case 'g':   txgain_dB   = atof(optarg);     break;
        case 'G':   uhd_txgain  = atof(optarg);     break;
        default:
            usage();
            return 0;
        }
    }

    unsigned int i;

    if (bandwidth > max_bandwidth) {
        fprintf(stderr,"error: %s, maximum bandwidth exceeded (%8.4f MHz)\n", argv[0], max_bandwidth*1e-6);
        return 0;
    } else if (bandwidth < min_bandwidth) {
        fprintf(stderr,"error: %s, minimum bandwidth exceeded (%8.4f kHz)\n", argv[0], min_bandwidth*1e-3);
        exit(1);
    } else if (cp_len == 0 || cp_len > M) {
        fprintf(stderr,"error: %s, cyclic prefix must be in (0,M]\n", argv[0]);
        exit(1);
    }

    uhd::device_addr_t dev_addr;
    //dev_addr["addr0"] = "192.168.10.2";
    //dev_addr["addr1"] = "192.168.10.3";
    uhd::usrp::single_usrp::sptr usrp = uhd::usrp::single_usrp::make(dev_addr);

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

    // initialize subcarrier allocation
    unsigned char p[M];
    ofdmframe_init_default_sctype(M, p);

    // create frame generator
    ofdmframegen fg = ofdmframegen_create(M, cp_len, taper_len, p);
    ofdmframegen_print(fg);

    // create modulator and pseudo-random number generator
    modem mod = modem_create(ms);
    msequence seq = msequence_create_default(8);

    // arrays
    unsigned int frame_len = M + cp_len;
    std::complex<float> buffer[frame_len];  // output time series
    std::complex<float> buffer_interp[frame_len];
    std::complex<float> buffer_resamp[frame_len];
    std::complex<float> X[M];

    // set up the metadta flags
    std::vector<std::complex<float> > buff(256);
    unsigned int tx_buffer_samples;
    uhd::tx_metadata_t md;
    md.start_of_burst = false;  // never SOB when continuous
    md.end_of_burst   = false;  // 
    md.has_time_spec  = false;  // set to false to send immediately

    // frame length: number of data symbols + 3 (for preamble)
    unsigned int num_symbols = 3 + num_data_symbols;

    // pilot symbols
    float pilots[8] = {  1.0f, -1.0f, -1.0f, 1.0f,
                        -1.0f,  1.0f, -1.0f, 1.0f };
    unsigned int pilot_index = 0;

    unsigned int pid;
    unsigned int j;
    tx_buffer_samples=0;
    for (pid=0; pid<num_frames; pid++) {

        // reset objects
        ofdmframegen_reset(fg); // reset frame generator (resets pilot generator, etc.)
        modem_reset(mod);       // modulator
        msequence_reset(seq);   // pseudo-random number generator
        pilot_index = 0;        // reset the pilot index to start at zero

        if (verbose)
            printf("tx packet id: %6u\n", pid);
        
        // generate frame
        for (i=0; i<num_symbols; i++) {
            if      (i==0) ofdmframegen_write_S0a(fg, buffer);  // first S0 symbol
            else if (i==1) ofdmframegen_write_S0b(fg, buffer);  // second S0 symbol
            else if (i==2) ofdmframegen_write_S1( fg, buffer);  // S1 symbol
            else {
                // generate data frame
                unsigned int bps = modem_get_bps(mod);
                for (j=0; j<M; j++) {
                    if (p[j] == OFDMFRAME_SCTYPE_DATA) {
                        // data subcarrier: generate pseudo-random number and
                        // modulate onto subcarrier
                        unsigned int sym = msequence_generate_symbol(seq, bps);
                        modem_modulate(mod, sym, &X[j]);

                        // DG: add your pilot precoder here
                        std::complex<float> pilot = pilots[pilot_index];
                        X[j] = 0.9f*X[j] + 0.1f*pilot;
                        pilot_index = (pilot_index + 1) % 8;
                    } else {
                        // pilot or NULL subcarrier (ignore)
                        X[j] = 0.0f;
                    }
                }

                // generate OFDM data symbol
                ofdmframegen_writesymbol(fg, X, buffer);
            }

            // interpolate by 2
            for (j=0; j<frame_len; j++)
                resamp2_crcf_interp_execute(interp, buffer[j], &buffer_interp[2*j]);
            
            // resample
            unsigned int nw;
            unsigned int n=0;
            for (j=0; j<2*frame_len; j++) {
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
    ofdmframegen_destroy(fg);
    resamp2_crcf_destroy(interp);
    resamp_crcf_destroy(resamp);
    modem_destroy(mod);


    return 0;
}

