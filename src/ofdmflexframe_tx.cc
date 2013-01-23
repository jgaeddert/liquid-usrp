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

void usage() {
    printf("ofdmflexframe_tx [OPTION]\n");
    printf("transmit OFDM packets\n");
    printf("\n");
    printf("  u,h   : usage/help\n");
    printf("  q/v   : quiet/verbose\n");
    printf("  f     : center frequency [Hz], default: 462 MHz\n");
    printf("  b     : bandwidth [Hz] (62.5kHz min, 8MHz max), default: 250 kHz\n");
    printf("  g     : software tx gain [dB] (default: -6dB)\n");
    printf("  G     : uhd tx gain [dB] (default: 40dB)\n");
    printf("  N     : number of frames, default: 1000\n");
    printf("  M     : number of subcarriers, default: 64\n");
    printf("  C     : cyclic prefix length, default: 16\n");
    printf("  T     : taper length, default: 0\n");
    printf("  P     : payload length [bytes], default: 800\n");
    printf("  m     : modulation scheme (qpsk default)\n");
    liquid_print_modulation_schemes();
    printf("  c     : coding scheme (inner): h74 default\n");
    printf("  k     : coding scheme (outer): none default\n");
    liquid_print_fec_schemes();
}

int main (int argc, char **argv)
{
    // command-line options
    bool verbose = true;

    double frequency = 462.0e6;         // carrier frequency
    double bandwidth = 600e3f;          // bandwidth
    unsigned int num_frames = 1000;     // number of frames to transmit
    double txgain_dB = -12.0f;          // software tx gain [dB]
    double uhd_txgain = 40.0;           // uhd (hardware) tx gain

    // ofdm properties
    unsigned int M = 48;                // number of subcarriers
    unsigned int cp_len = 6;            // cyclic prefix length
    unsigned int taper_len = 4;         // taper length

    modulation_scheme ms = LIQUID_MODEM_QPSK;// modulation scheme
    unsigned int payload_len = 800;         // original data message length
    crc_scheme check = LIQUID_CRC_32;       // data validity check
    fec_scheme fec0 = LIQUID_FEC_NONE;      // fec (inner)
    fec_scheme fec1 = LIQUID_FEC_GOLAY2412; // fec (outer)
    
    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:g:G:N:M:C:T:P:m:c:k:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose     = false;            break;
        case 'v':   verbose     = true;             break;
        case 'f':   frequency   = atof(optarg);     break;
        case 'b':   bandwidth   = atof(optarg);     break;
        case 'g':   txgain_dB   = atof(optarg);     break;
        case 'G':   uhd_txgain  = atof(optarg);     break;
        case 'N':   num_frames  = atoi(optarg);     break;
        case 'M':   M           = atoi(optarg);     break;
        case 'C':   cp_len      = atoi(optarg);     break;
        case 'T':   taper_len   = atoi(optarg);     break;
        case 'P':   payload_len = atoi(optarg);     break;
        case 'm':   ms          = liquid_getopt_str2mod(optarg);    break;
        case 'c':   fec0        = liquid_getopt_str2fec(optarg);    break;
        case 'k':   fec1        = liquid_getopt_str2fec(optarg);    break;
        default:    usage();                        return 0;
        }
    }

    unsigned int i;

    if (cp_len == 0 || cp_len > M) {
        fprintf(stderr,"error: %s, cyclic prefix must be in (0,M]\n", argv[0]);
        exit(1);
    } else if (ms == LIQUID_MODEM_UNKNOWN) {
        fprintf(stderr,"error: %s, unknown/unsupported mod. scheme\n", argv[0]);
        exit(-1);
    }

    uhd::device_addr_t dev_addr;
    //dev_addr["addr0"] = "192.168.10.2";
    //dev_addr["addr1"] = "192.168.10.3";
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);

    // try to set tx rate (oversampled to compensate for CIC filter)
    usrp->set_tx_rate(3.0f * bandwidth);

    // get actual tx rate
    double usrp_tx_rate = usrp->get_tx_rate();

    // compute arbitrary resampling rate (make up the difference in software)
    double tx_resamp_rate = usrp_tx_rate / bandwidth;

    usrp->set_tx_freq(frequency);
    usrp->set_tx_gain(uhd_txgain);

    printf("frequency       :   %10.4f [MHz]\n", frequency*1e-6f);
    printf("bandwidth       :   %10.4f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity       :   %s\n", (verbose?"enabled":"disabled"));
    printf("usrp sample rate:   %10.4f kHz = %10.4f kHz * %8.6f\n",
            usrp_tx_rate * 1e-3f,
            bandwidth    * 1e-3f,
            tx_resamp_rate);

    // set the IF filter bandwidth
    //usrp->set_tx_bandwidth(2.0f*bandwidth);

    // add arbitrary resampling component
    msresamp_crcf resamp = msresamp_crcf_create(tx_resamp_rate, 60.0f);

    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/20.0f);

    // data arrays
    unsigned char header[8];
    unsigned char payload[payload_len];
    
    // create frame generator (default subcarrier allocation)
    ofdmflexframegenprops_s fgprops;
    ofdmflexframegenprops_init_default(&fgprops);
    fgprops.check           = check;
    fgprops.fec0            = fec0;
    fgprops.fec1            = fec1;
    fgprops.mod_scheme      = ms;
    ofdmflexframegen fg = ofdmflexframegen_create(M, cp_len, taper_len, NULL, &fgprops);
    ofdmflexframegen_print(fg);

    // allocate array to hold each OFDM symbol
    unsigned int symbol_len = M + cp_len;   // ofdm symbol length (samples)
    std::complex<float> ofdm_symbol[symbol_len]; // output time series of each symbol

    // create buffer for arbitrary resamper output
    std::complex<float> buffer_resamp[(int)(2*tx_resamp_rate) + 64];

    // vector buffer to send data to USRP
    std::vector<std::complex<float> > usrp_buffer(256);
    unsigned int usrp_sample_counter = 0;

    // set up the metadta flags
    uhd::tx_metadata_t md;
    md.start_of_burst = false;  // never SOB when continuous
    md.end_of_burst   = false;  // 
    md.has_time_spec  = false;  // set to false to send immediately

    unsigned int j;
    unsigned int pid;
    for (pid=0; pid<num_frames; pid++) {
        // reset frame generator (resets pilot generator, etc.)
        ofdmflexframegen_reset(fg);

        if (verbose)
            printf("tx packet id: %6u\n", pid);
        
        // write header (first two bytes packet ID, remaining are random)
        header[0] = (pid >> 8) & 0xff;
        header[1] = (pid     ) & 0xff;
        for (j=2; j<8; j++)
            header[j] = rand() & 0xff;

        // initialize payload
        for (j=0; j<payload_len; j++)
            payload[j] = rand() & 0xff;

        // assemble frame
        ofdmflexframegen_assemble(fg, header, payload, payload_len);

        // generate a single OFDM frame
        int last_symbol=0;
        unsigned int zero_pad=1;
        while (!last_symbol || zero_pad > 0) {

            // generate OFDM symbol
            if (!last_symbol) {
                // generate symbol
                last_symbol = ofdmflexframegen_writesymbol(fg, ofdm_symbol);
            } else {
                zero_pad--;
                for (j=0; j<symbol_len; j++)
                    ofdm_symbol[j] = 0.0f;
            }

            // resample OFDM symbol and push resulting samples to USRP
            for (j=0; j<symbol_len; j++) {
                // resample OFDM symbol one sample at a time
                unsigned int nw;    // number of samples output from resampler
                msresamp_crcf_execute(resamp, &ofdm_symbol[j], 1, buffer_resamp, &nw);

                // for each output sample, stuff into USRP buffer
                unsigned int n;
                for (n=0; n<nw; n++) {
                    // append to USRP buffer, scaling by software
                    usrp_buffer[usrp_sample_counter++] = g*buffer_resamp[n];

                    // once USRP buffer is full, reset counter and send to device
                    if (usrp_sample_counter==256) {
                        // reset counter
                        usrp_sample_counter=0;

                        // send the result to the USRP
                        usrp->get_device()->send(
                            &usrp_buffer.front(), usrp_buffer.size(), md,
                            uhd::io_type_t::COMPLEX_FLOAT32,
                            uhd::device::SEND_MODE_FULL_BUFF
                        );
                    }
                }
            }

        } // while loop


    } // packet loop
 
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

    // delete allocated objects
    ofdmflexframegen_destroy(fg);
    msresamp_crcf_destroy(resamp);

    return 0;
}

