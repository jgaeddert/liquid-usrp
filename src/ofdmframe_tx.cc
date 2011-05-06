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
    printf("ofdmframe_tx -- transmit OFDM packets\n");
    printf("  u,h   :   usage/help\n");
    printf("  q/v   :   quiet/verbose\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  b     :   bandwidth [Hz] (62.5kHz min, 8MHz max)\n");
    printf("  M     :   number of subcarriers, default: 64\n");
    printf("  C     :   cyclic prefix length, default: 16\n");
    printf("  m     :   modulation scheme: psk, dpsk, ask, <qam>, apsk\n");
    printf("  p     :   modulation depth [bits/symbol], default: 2\n");
    printf("  c     :   fec coding scheme (inner)\n");
    printf("  k     :   fec coding scheme (outer)\n");
    // print all available FEC schemes
    unsigned int i;
    for (i=0; i<LIQUID_FEC_NUM_SCHEMES; i++)
        printf("          [%s] %s\n", fec_scheme_str[i][0], fec_scheme_str[i][1]);
}

int main (int argc, char **argv)
{
    // command-line options
    bool verbose = true;

    double min_bandwidth = 0.25*(64e6 / 512.0);
    double max_bandwidth = 0.25*(64e6 /   4.0);

    double frequency = 462.0e6;
    double bandwidth = min_bandwidth;
    //float num_seconds = 5.0f;

    unsigned int M = 64;                // number of subcarriers
    unsigned int cp_len = 16;           // cyclic prefix length
    unsigned int num_symbols_S0 = 2;    // number of S0 symbols
    unsigned int num_symbols_S1 = 2;    // number of S0 symbols

    modulation_scheme ms = LIQUID_MODEM_QAM;     // modulation scheme
    unsigned int bps = 2;               // modulation depth
    unsigned int packet_len_dec = 50;   // original data message length
    crc_scheme check = LIQUID_CRC_32;          // data validity check
    fec_scheme fec0 = LIQUID_FEC_NONE;         // fec (inner)
    fec_scheme fec1 = LIQUID_FEC_HAMMING128;   // fec (outer)

    double txgain_dB = -3.0f;

    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:M:C:m:p:c:k:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose = false;                break;
        case 'v':   verbose = true;                 break;
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 'M':   M = atoi(optarg);               break;
        case 'C':   cp_len = atoi(optarg);          break;
        case 'm':
            ms = liquid_getopt_str2mod(optarg);
            if (ms == LIQUID_MODEM_UNKNOWN) {
                fprintf(stderr, "error: %s unknown/unsupported mod. scheme: %s\n", argv[0], optarg);
                ms = LIQUID_MODEM_UNKNOWN;
            }
            break;
        case 'p':   bps = atoi(optarg);             break;
        case 'c':   fec0 = liquid_getopt_str2fec(optarg);   break;
        case 'k':   fec1 = liquid_getopt_str2fec(optarg);   break;
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
    usrp->set_tx_gain(-40);
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
    unsigned int p[M];
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

    // number of ofdm data symbols in frame (excluding preamble)
    unsigned int num_symbols_data = (2800/M_data) + 1;

    // total number of modulated symbols in frame
    unsigned int num_mod_symbols = num_symbols_data * M_data;
    
    // create packet generator
    bpacketgen pg = bpacketgen_create(0, packet_len_dec, check, fec0, fec1);
    bpacketgen_print(pg);
    unsigned int packet_len_enc = bpacketgen_get_packet_len(pg);

    // data arrays
    unsigned char packet_dec[packet_len_dec];
    unsigned char packet_enc[packet_len_enc];
    
    // 
    div_t dsyms = div(8*packet_len_enc,bps);
    unsigned int num_packet_symbols = dsyms.quot + (dsyms.rem ? 1 : 0);
    unsigned int packets_per_frame = num_mod_symbols / num_packet_symbols;

    printf("ofdmframe_tx properties\n");
    printf("    null            :   %u\n", M_null);
    printf("    plot            :   %u\n", M_pilot);
    printf("    data            :   %u\n", M_data);
    printf("    ofdm syms/frame :   %u\n", num_symbols_data);
    printf("    mod. syms/frame :   %u\n", num_mod_symbols);
    printf("    packet len dec  :   %u bytes\n", packet_len_dec);
    printf("    packet len enc  :   %u bytes\n", packet_len_enc);
    printf("    mod. syms/packet:   %u\n", num_packet_symbols);
    printf("    packets/frame   :   %u\n", packets_per_frame);

    // create frame generator
    ofdmframegen fg = ofdmframegen_create(M, cp_len, p);
    ofdmframegen_print(fg);

    // arrays
    std::complex<float> modsyms[num_packet_symbols];
    std::complex<float> X[M];           // channelized symbols
    std::complex<float> S0[M];          // PLCP sequence (short)
    std::complex<float> S1[M];          // PLCP sequence (long)
    std::complex<float> y[M+cp_len];    // output time series

    // full frame
    unsigned int frame_len = num_symbols_S0 * M +
                             num_symbols_S1 * M + cp_len +
                             num_symbols_data*(M+cp_len);
    std::complex<float> frame[frame_len];
    std::complex<float> buffer_interp[2*frame_len];
    std::complex<float> buffer_resamp[3*frame_len];

    // initialize sequence arrays
    ofdmframegen_write_S0(fg, S0);
    ofdmframegen_write_S1(fg, S1);

    // create modem
    modem mod = modem_create(ms,bps);

    // set up the metadta flags
    std::vector<std::complex<float> > buff(M+cp_len);
    uhd::tx_metadata_t md;
    md.start_of_burst = false;  // never SOB when continuous
    md.end_of_burst   = false;  // 
    md.has_time_spec  = false;  // set to false to send immediately

    unsigned int num_frames = -1;

    unsigned int j;
    unsigned int k;
    unsigned int num_samples=0;
    for (i=0; i<num_frames; i++) {

        //
        // preamble
        //
        num_samples=0;

        // write short sequence(s)
        for (j=0; j<num_symbols_S0; j++) {
            memmove(&frame[num_samples], S0, M*sizeof(std::complex<float>));
            num_samples += M;
        }

        // write long sequence extension
        memmove(&frame[num_samples], &S1[M-cp_len], cp_len*sizeof(std::complex<float>));
        num_samples += cp_len;

        // write long sequence(s)
        for (j=0; j<num_symbols_S1; j++) {
            memmove(&frame[num_samples], S1, M*sizeof(std::complex<float>));
            num_samples += M;
        }

        //
        // payload
        //

        // payload modem symbol counter
        unsigned int k=0;           // modem symbol counter
        unsigned int n;
        unsigned int num_packets=0; // packet counter

        // generate ofdm payload symbols
        for (n=0; n<num_symbols_data; n++) {

            // load data onto subcarriers
            for (j=0; j<M; j++) {

                // ignore non-data subcarriers
                if (p[j] != OFDMFRAME_SCTYPE_DATA)
                    continue;

                // generate payload
                if (k==0 && num_packets < packets_per_frame) {
                    unsigned int ii;
                    for (ii=0; ii<packet_len_dec; ii++)
                        packet_dec[ii] = rand() & 0xff;

                    // encode packet
                    bpacketgen_encode(pg, packet_dec, packet_enc);

                    // generate data symbols
                    for (ii=0; ii<num_packet_symbols; ii++) {
                        unsigned char sym=0;
                        liquid_unpack_array(packet_enc,
                                            packet_len_enc,
                                            ii*bps,     // bit index
                                            bps,        // symbol size
                                            &sym);      // output symbol

                        // modulate symbol
                        modem_modulate(mod, sym, &modsyms[ii]);
                    }
                    num_packets++;
                } else if (k==0 && num_packets >= packets_per_frame) {
                    //printf("random payload\n");
                    unsigned int ii;
                    for (ii=0; ii<num_packet_symbols; ii++) {
                        unsigned char sym = modem_gen_rand_sym(mod);
                        modem_modulate(mod, sym, &modsyms[ii]);
                    }
                }

                // store symbol to ofdm input subcarrier array
                X[j] = modsyms[k];

                // update counter
                k = (k+1) % num_packet_symbols;
            }

            // generate ofdm payload symbol
            ofdmframegen_writesymbol(fg, X, y);

            //gport_produce(port_tx, (void*)y, M+cp_len);
            memmove(&frame[num_samples], y, (M+cp_len)*sizeof(std::complex<float>));
            num_samples += M+cp_len;
        }

        // pad remaining samples with zeros
        for (j=num_samples; j<frame_len; j++)
            frame[j] = 0.0f;

        // interpolate by 2
        for (j=0; j<frame_len; j++) {
            //
            resamp2_crcf_interp_execute(interp, frame[j], &buffer_interp[2*j]);
        }
        
        // resample
        unsigned int nw;
        n=0;
        for (j=0; j<2*frame_len; j++) {
            resamp_crcf_execute(resamp, buffer_interp[j], &buffer_resamp[n], &nw);
            n += nw;
        }

        buff.resize(n);
        for (j=0; j<n; j++) {
            buff[j] = g*buffer_resamp[j];
        }

        //send the entire contents of the buffer
        usrp->get_device()->send(
            &buff.front(), buff.size(), md,
            uhd::io_type_t::COMPLEX_FLOAT32,
            uhd::device::SEND_MODE_FULL_BUFF
        );

        // reset frame generator (resets pilot generator, etc.)
        ofdmframegen_reset(fg);

        if (verbose)
            printf("frame transmitted %2u / %2u packets\n", num_packets, packets_per_frame);
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
    ofdmframegen_destroy(fg);
    modem_destroy(mod);
    bpacketgen_destroy(pg);
    resamp2_crcf_destroy(interp);
    resamp_crcf_destroy(resamp);
    return 0;
}

