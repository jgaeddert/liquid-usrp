/*
 * Copyright (c) 2013 Joseph Gaeddert
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

#include "multichanneltx.h"

void usage() {
    printf("multichannel_tx [OPTION]\n");
    printf("\n");
    printf("  h     : usage/help\n");
    printf("  q/v   : quiet/verbose\n");
    printf("  f     : center frequency [Hz],  default: 462 MHz\n");
    printf("  b     : channel bandwidth [Hz], default: 200 kHz\n");
    printf("  M     : number of subcarriers,  default:  48\n");
    printf("  C     : cyclic prefix length,   default:   6\n");
    printf("  T     : taper length,           default:   4\n");
    printf("  P     : payload length [bytes], default: 1200 bytes\n");
    printf("  n     : number of channels,     default: 1\n");
    printf("  g     : software tx gain [dB],  default: -10 dB\n");
    printf("  G     : uhd tx gain [dB],       default: 40 dB\n");
    printf("  m     : modulation scheme,      default: qpsk\n");
    liquid_print_modulation_schemes();
    printf("  c     : coding scheme (inner),  default: g2412\n");
    printf("  k     : coding scheme (outer),  default: none\n");
    liquid_print_fec_schemes();
}

int main (int argc, char **argv)
{
    // command-line options
    bool verbose = true;

    double frequency = 462.0e6;         // center frequency [Hz]
    double bandwidth = 250e3f;          // channel bandwidth [Hz]
    unsigned int num_channels = 1;      // number of channels
    double txgain_dB = -10.0f;          // software tx gain [dB]
    double uhd_txgain = 40.0;           // uhd (hardware) tx gain

    unsigned int payload_len = 1200;    // original data message length

    // ofdm properties
    unsigned int M          = 48;       // number of subcarriers
    unsigned int cp_len     =  6;       // cyclic prefix length
    unsigned int taper_len  =  4;       // cyclic prefix length

    //crc_scheme check     = LIQUID_CRC_32;        // data validity check
    modulation_scheme ms = LIQUID_MODEM_QPSK;    // modulation scheme
    fec_scheme fec0      = LIQUID_FEC_NONE;      // fec (inner)
    fec_scheme fec1      = LIQUID_FEC_HAMMING128;// fec (outer)

    //
    int d;
    while ((d = getopt(argc,argv,"hqvf:b:M:C:T:P:n:g:G:m:c:k:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose     = false;            break;
        case 'v':   verbose     = true;             break;
        case 'f':   frequency   = atof(optarg);     break;
        case 'b':   bandwidth   = atof(optarg);     break;
        case 'M':   M           = atoi(optarg);     break;
        case 'C':   cp_len      = atoi(optarg);     break;
        case 'T':   taper_len   = atoi(optarg);     break;
        case 'P':   payload_len = atoi(optarg);     break;
        case 'n':   num_channels= atoi(optarg);     break;
        case 'g':   txgain_dB   = atof(optarg);     break;
        case 'G':   uhd_txgain  = atof(optarg);     break;
        case 'm':   ms          = liquid_getopt_str2mod(optarg);    break;
        case 'c':   fec0        = liquid_getopt_str2fec(optarg);    break;
        case 'k':   fec1        = liquid_getopt_str2fec(optarg);    break;
        default:    usage();                        return 0;
        }
    }

    unsigned int i;

    uhd::device_addr_t dev_addr;
    //dev_addr["addr0"] = "192.168.10.2";
    //dev_addr["addr1"] = "192.168.10.3";
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);

    // set properties
    // TODO: compensate for rate provided by multichanneltx
    double tx_rate = num_channels*bandwidth;

    // try to set tx rate
    usrp->set_tx_rate(2.0f*tx_rate);

    // get actual tx rate
    double usrp_tx_rate = usrp->get_tx_rate();

    // compute arbitrary resampling rate
    double tx_resamp_rate = usrp_tx_rate / tx_rate;

    usrp->set_tx_freq(frequency);
    usrp->set_tx_gain(uhd_txgain);

    printf("frequency       :   %10.4f [MHz]\n", frequency*1e-6f);
    printf("channels        :   %10.4f kHz x %u\n", bandwidth * 1e-3f, num_channels);
    printf("usrp sample rate:   %10.4f kHz = %10.4f kHz * %8.6f\n",
            usrp_tx_rate * 1e-3f,
            tx_rate      * 1e-3f,
            tx_resamp_rate);
    printf("verbosity       :   %s\n", (verbose?"enabled":"disabled"));

    // set the IF filter bandwidth
    //usrp->set_tx_bandwidth(2.0f*tx_rate);

    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/20.0f);
    g /= (float)num_channels;

    // data arrays
    unsigned char header[8];
    unsigned char payload[payload_len];
    unsigned int pid[num_channels];
    for (i=0; i<num_channels; i++)
        pid[i] = 0;
    
    // create multichannel transmitter object
    multichanneltx mctx(num_channels, M, cp_len, taper_len);
    
    // allocate array to hold samples
    unsigned int mctx_buffer_len = 2*num_channels;
    std::complex<float> mctx_buffer[mctx_buffer_len];

    // vector buffer to send data to USRP
    std::vector<std::complex<float> > usrp_buffer(256);
    unsigned int usrp_sample_counter = 0;

    // set up the metadta flags
    uhd::tx_metadata_t md;
    md.start_of_burst = false;  // never SOB when continuous
    md.end_of_burst   = false;  // 
    md.has_time_spec  = false;  // set to false to send immediately

    int continue_running = 1;
    while (continue_running) {
        // generate data if necessary
        unsigned int channel_id;
        for (channel_id=0; channel_id<num_channels; channel_id++) {
            if (mctx.IsChannelReadyForData(channel_id)) {
                // increment id
                pid[channel_id]++;

                // write header (first two bytes packet ID, then channel id, then random)
                header[0] = (pid[channel_id] >> 8) & 0xff;
                header[1] = (pid[channel_id]     ) & 0xff;
                header[2] = channel_id             & 0xff;
                for (i=3; i<8; i++)
                    header[i] = rand() & 0xff;

                // initialize payload
                for (i=0; i<payload_len; i++)
                    payload[i] = rand() & 0xff;

#if 0
                // update payload data (random length)
                //mctx.UpdateData(channel_id, header, payload, rand() % payload_len);
#else
                // update payload data
                mctx.UpdateData(channel_id, header, payload, payload_len, ms, fec0, fec1);
#endif
            }
        }

        // generate samples
        mctx.GenerateSamples(mctx_buffer);


        // push resulting samples to USRP
        for (i=0; i<mctx_buffer_len; i++) {

            // append to USRP buffer, scaling by software
            usrp_buffer[usrp_sample_counter++] = g*mctx_buffer[i];

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

    } // while loop

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

    return 0;
}

