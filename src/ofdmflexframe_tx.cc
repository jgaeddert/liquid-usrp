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

void usage() {
    printf("ofdmflexframe_tx [OPTION]\n");
    printf("transmit OFDM packets\n");
    printf("\n");
    printf("  u,h   : usage/help\n");
    printf("  q/v   : quiet/verbose\n");
    printf("  f     : center frequency [Hz]\n");
    printf("  b     : bandwidth [Hz] (62.5kHz min, 8MHz max)\n");
    printf("  g     : software tx gain [dB] (default: -6dB)\n");
    printf("  G     : uhd tx gain [dB] (default: -40dB)\n");
    printf("  N     : number of frames, default: 1000\n");
    printf("  M     : number of subcarriers, default: 64\n");
    printf("  C     : cyclic prefix length, default: 16\n");
    printf("  P     : payload length [bytes], default: 256\n");
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

    double min_bandwidth = 0.25*(64e6 / 512.0);
    double max_bandwidth = 0.25*(64e6 /   4.0);

    double frequency = 462.0e6;
    double bandwidth = min_bandwidth;
    unsigned int num_frames = 1000;     // number of frames to transmit
    double txgain_dB = -6.0f;           // software tx gain [dB]
    double uhd_txgain = -40.0;          // uhd (hardware) tx gain


    unsigned int M = 64;                // number of subcarriers
    unsigned int cp_len = 16;           // cyclic prefix length
    unsigned int num_symbols_S0 = 3;    // number of S0 symbols

    modulation_scheme ms = LIQUID_MODEM_QAM;     // modulation scheme
    unsigned int bps = 2;               // modulation depth
    unsigned int payload_len = 256;     // original data message length
    crc_scheme check = LIQUID_CRC_32;          // data validity check
    fec_scheme fec0 = LIQUID_FEC_NONE;         // fec (inner)
    fec_scheme fec1 = LIQUID_FEC_HAMMING128;   // fec (outer)

    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:g:G:N:M:C:P:m:c:k:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose = false;                break;
        case 'v':   verbose = true;                 break;
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 'g':   txgain_dB = atof(optarg);       break;
        case 'G':   uhd_txgain = atof(optarg);      break;
        case 'N':   num_frames = atoi(optarg);      break;
        case 'M':   M = atoi(optarg);               break;
        case 'C':   cp_len = atoi(optarg);          break;
        case 'P':   payload_len = atoi(optarg);     break;
        case 'm':
            liquid_getopt_str2modbps(optarg, &ms, &bps);
            if (ms == LIQUID_MODEM_UNKNOWN) {
                fprintf(stderr,"error: %s, unknown/unsupported mod. scheme: %s\n", argv[0], optarg);
                exit(-1);
            }
            break;
        case 'c':
            // inner FEC scheme
            fec0 = liquid_getopt_str2fec(optarg);
            if (fec0 == LIQUID_FEC_UNKNOWN) {
                fprintf(stderr,"error: unknown/unsupported inner FEC scheme \"%s\"\n\n",optarg);
                exit(1);
            }
            break;
        case 'k':
            // outer FEC scheme
            fec1 = liquid_getopt_str2fec(optarg);
            if (fec1 == LIQUID_FEC_UNKNOWN) {
                fprintf(stderr,"error: unknown/unsupported outer FEC scheme \"%s\"\n\n",optarg);
                exit(1);
            }
            break;
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

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
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
    usrp->set_tx_gain(uhd_txgain);
    // set the IF filter bandwidth
    //usrp->set_tx_bandwidth(2.0f*tx_rate);


    // add arbitrary resampling component
    resamp_crcf resamp = resamp_crcf_create(tx_resamp_rate,7,0.4f,60.0f,64);
    resamp_crcf_setrate(resamp, tx_resamp_rate);

    // half-band resampler
    resamp2_crcf interp = resamp2_crcf_create(7,0.0f,40.0f);

    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/10.0f);
 

    // initialize subcarrier allocation
    unsigned char p[M];
    unsigned int guard = M / 6;
    unsigned int pilot_spacing = 8;
    unsigned int i0 = (M/2) - guard;
    unsigned int i1 = (M/2) + guard;
    for (i=0; i<M; i++) {
        if ( i == 0 || (i > i0 && i < i1) )
            p[i] = OFDMFRAME_SCTYPE_NULL;
        else if ( (i%pilot_spacing)==0 )
            p[i] = OFDMFRAME_SCTYPE_PILOT;
        else
            p[i] = OFDMFRAME_SCTYPE_DATA;
    }

    unsigned int M_null=0;
    unsigned int M_pilot=0;
    unsigned int M_data=0;
    ofdmframe_validate_sctype(p,M, &M_null, &M_pilot, &M_data);

    // data arrays
    unsigned char header[8];
    unsigned char payload[payload_len];
    
    // create frame generator
    ofdmflexframegenprops_s fgprops;
    ofdmflexframegenprops_init_default(&fgprops);
    fgprops.num_symbols_S0  = num_symbols_S0;
    fgprops.payload_len     = payload_len;
    fgprops.check           = check;
    fgprops.fec0            = fec0;
    fgprops.fec1            = fec1;
    fgprops.mod_scheme      = ms;
    fgprops.mod_bps         = bps;
    ofdmflexframegen fg = ofdmflexframegen_create(M, cp_len, p, &fgprops);
    ofdmflexframegen_print(fg);

    // arrays
    std::complex<float> buffer[M+cp_len];    // output time series
    std::complex<float> buffer_interp[2*(M+cp_len)];
    std::complex<float> buffer_resamp[3*(M+cp_len)];

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
        ofdmflexframegen_assemble(fg, header, payload);

        // generate frame
        int last_symbol=0;
        unsigned int zero_pad=1;
        unsigned int num_samples;
        while (!last_symbol || zero_pad > 0) {
            if (!last_symbol) {
                // generate symbol
                last_symbol = ofdmflexframegen_writesymbol(fg, buffer, &num_samples);
            } else {
                zero_pad--;
                num_samples = M+cp_len;
                for (j=0; j<num_samples; j++)
                    buffer[j] = 0.0f;
            }

            // interpolate by 2
            for (j=0; j<num_samples; j++)
                resamp2_crcf_interp_execute(interp, buffer[j], &buffer_interp[2*j]);
            
            // resample
            unsigned int nw;
            unsigned int n=0;
            for (j=0; j<2*num_samples; j++) {
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
    ofdmflexframegen_destroy(fg);
    resamp2_crcf_destroy(interp);
    resamp_crcf_destroy(resamp);

    return 0;
}

