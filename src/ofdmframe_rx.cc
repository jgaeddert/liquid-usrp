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
 
static bool verbose;

// common data structure
struct commondata_s {
    unsigned int symbols_per_frame; // number of ofdm symbols in the frame
    unsigned int num_symbols_received;

    // objects
    modem demod;    // demodulator
    bpacketsync ps; // packet synchronizer

    // data counter
    unsigned int num_packets_received;
    unsigned int num_valid_packets_received;
    unsigned int num_valid_bytes_received;
};

static int packet_callback(unsigned char * _payload,
                           int _payload_valid,
                           unsigned int _payload_len,
                           void * _userdata)
{
    // type cast userdata
    struct commondata_s * q = (struct commondata_s *) _userdata;

    if (verbose)
        printf("  packet received\n");

    q->num_packets_received++;
    if ( !_payload_valid ) {
        if (verbose) printf("payload crc : FAIL\n");
    } else {
        q->num_valid_packets_received++;
        q->num_valid_bytes_received += _payload_len;
    }
    return 0;
}

static int ofdm_callback(std::complex<float> * _X,
                         unsigned int * _p,
                         unsigned int _M,
                         void * _userdata)
{
    // type cast userdata
    struct commondata_s * q = (struct commondata_s *) _userdata;

    // get demodulator depth
    unsigned int bps = modem_get_bps(q->demod);

    // run demodulator, pushing resulting bits to packet synchronizer
    unsigned int i;
    for (i=0; i<_M; i++) {
        if (_p[i] == OFDMFRAME_SCTYPE_DATA) {
            // run demodulator
            unsigned int demod_sym;
            modem_demodulate(q->demod, _X[i], &demod_sym);

            // push through packet synchronizer
            bpacketsync_execute_sym(q->ps, demod_sym, bps);
        }
    }

    //printf("**** callback invoked\n");
    q->num_symbols_received++;

    if (q->num_symbols_received == q->symbols_per_frame) {
        q->num_symbols_received = 0;
        if (verbose)
            printf("**** frame received\n");

        // tell ofdm frame synchronizer to reset
        return 1;
    }
    return 0;
}

void usage() {
    printf("ofdmframe_rx -- receive OFDM packets\n");
    printf("  u,h   :   usage/help\n");
    printf("  q/v   :   quiet/verbose\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  b     :   bandwidth [Hz]\n");
    printf("  M     :   number of subcarriers, default: 64\n");
    printf("  C     :   cyclic prefix length, default: 16\n");
    printf("  t     :   run time [seconds]\n");
    printf("  m     :   modulation scheme: psk, dpsk, ask, <qam>, apsk\n");
    printf("  p     :   modulation depth [bits/symbol], default: 2\n");
}

int main (int argc, char **argv)
{
    // command-line options
    verbose = false;

    double min_bandwidth = 0.25*(64e6 / 512.0);
    double max_bandwidth = 0.25*(64e6 /   4.0);

    double frequency = 462.0e6;
    double bandwidth = min_bandwidth;
    double num_seconds = 5.0f;

    // 
    unsigned int M = 64;                // number of subcarriers
    unsigned int cp_len = 16;           // cyclic prefix length

    modulation_scheme ms = LIQUID_MODEM_QAM;
    unsigned int bps = 2;

    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:M:C:t:m:p:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'q':   verbose = false;                break;
        case 'v':   verbose = true;                 break;
        case 'f':   frequency = atof(optarg);       break;
        case 'b':   bandwidth = atof(optarg);       break;
        case 'M':   M = atoi(optarg);               break;
        case 'C':   cp_len = atoi(optarg);          break;
        case 't':   num_seconds = atof(optarg);     break;
        case 'm':
            ms = liquid_getopt_str2mod(optarg);
            if (ms == LIQUID_MODEM_UNKNOWN) {
                fprintf(stderr, "error: %s unknown/unsupported mod. scheme: %s\n", argv[0], optarg);
                ms = LIQUID_MODEM_UNKNOWN;
            }
            break;
        case 'p':   bps = atoi(optarg);             break;
        default:
            usage();
            return 0;
        }
    }

    unsigned int i;

    if (bandwidth > max_bandwidth) {
        fprintf(stderr,"error: %s, maximum symbol rate exceeded (%8.4f MHz)\n", argv[0], max_bandwidth*1e-6);
        exit(1);
    } else if (bandwidth < min_bandwidth) {
        fprintf(stderr,"error: %s, minimum symbol rate exceeded (%8.4f kHz)\n", argv[0], min_bandwidth*1e-3);
        exit(1);
    } else if (cp_len == 0 || cp_len > M) {
        fprintf(stderr,"error: %s, cyclic prefix must be in (0,M]\n", argv[0]);
        exit(1);
    }

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);

    stream_cmd.stream_now = true;

    uhd::device_addr_t dev_addr;
    uhd::usrp::single_usrp::sptr usrp = uhd::usrp::single_usrp::make(dev_addr);

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    // set properties
    double rx_rate = 4.0f*bandwidth;
#if 0
    usrp->set_rx_rate(rx_rate);
#else
    // NOTE : the sample rate computation MUST be in double precision so
    //        that the UHD can compute its decimation rate properly
    unsigned int decim_rate = (unsigned int)(64e6 / rx_rate);
    // ensure multiple of 2
    decim_rate = (decim_rate >> 1) << 1;
    // compute usrp sampling rate
    double usrp_rx_rate = 64e6 / (float)decim_rate;
    // compute arbitrary resampling rate
    double rx_resamp_rate = rx_rate / usrp_rx_rate;
    printf("sample rate :   %12.8f kHz = %12.8f * %8.6f (decim %u)\n",
            rx_rate * 1e-3f,
            usrp_rx_rate * 1e-3f,
            rx_resamp_rate,
            decim_rate);
    usrp->set_rx_rate(usrp_rx_rate);
#endif
    usrp->set_rx_freq(frequency);
    usrp->set_rx_gain(40);

    // add arbitrary resampling block
    resamp_crcf resamp = resamp_crcf_create(rx_resamp_rate,7,0.4f,60.0f,64);
    resamp_crcf_setrate(resamp, rx_resamp_rate);

    const size_t max_samps_per_packet = usrp->get_device()->get_max_recv_samps_per_packet();
    unsigned int num_blocks = (unsigned int)((rx_rate*num_seconds)/(max_samps_per_packet));

    //allocate recv buffer and metatdata
    uhd::rx_metadata_t md;
    std::vector<std::complex<float> > buff(max_samps_per_packet);

    // half-band decimator
    resamp2_crcf decim = resamp2_crcf_create(7,0.0f,40.0f);

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

    // create common data object
    struct commondata_s userdata;

    // create frame synchronizer
    ofdmframesync fs = ofdmframesync_create(M, cp_len, p, ofdm_callback, (void*)&userdata);
    ofdmframesync_print(fs);

    // create demodulator
    modem demod = modem_create(ms, bps);

    // create packet synchronizer
    bpacketsync ps = bpacketsync_create(0, packet_callback, (void*)&userdata);
 
    // initialize common data object
    userdata.symbols_per_frame      = num_symbols_data;
    userdata.num_symbols_received   = 0;
    userdata.demod                  = demod;
    userdata.ps                     = ps;
    userdata.num_packets_received   = 0;
    userdata.num_valid_packets_received = 0;
    userdata.num_valid_bytes_received = 0;

    // start data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    printf("usrp data transfer started\n");
 
    std::complex<float> data_rx[64];
    std::complex<float> data_decim[32];
    std::complex<float> data_resamp[64];
 
    unsigned int n=0;
    for (i=0; i<num_blocks; i++) {
        // grab data from port
        size_t num_rx_samps = usrp->get_device()->recv(
            &buff.front(), buff.size(), md,
            uhd::io_type_t::COMPLEX_FLOAT32,
            uhd::device::RECV_MODE_ONE_PACKET
        );

        //handle the error codes
        switch(md.error_code){
        case uhd::rx_metadata_t::ERROR_CODE_NONE:
        case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
            break;

        default:
            std::cerr << "Error code: " << md.error_code << std::endl;
            std::cerr << "Unexpected error on recv, exit test..." << std::endl;
            return 1;
        }

        // for now copy vector "buff" to array of complex float
        // TODO : apply bandwidth-dependent gain
        unsigned int j;
        unsigned int nw=0;
        for (j=0; j<num_rx_samps; j++) {
#if 0
            //data_rx[j] = buff[j];
            std::complex<float> sample = buff[j];
            resamp_crcf_execute(resamp, sample, &data_rx[n], &nw);
            n += nw;
#endif
            // push 64 samples into buffer
            data_rx[n++] = buff[j];

            if (n==64) {
                // reset counter
                n=0;

                // decimate to 32
                unsigned int k;
                for (k=0; k<32; k++)
                    resamp2_crcf_decim_execute(decim, &data_rx[2*k], &data_decim[k]);

                // apply resampler
                for (k=0; k<32; k++) {
                    resamp_crcf_execute(resamp, data_decim[k], &data_resamp[n], &nw);
                    n += nw;
                }

                // push through synchronizer
                ofdmframesync_execute(fs, data_resamp, n);

                // reset counter (again)
                n = 0;
            }

        }

#if 0
        for (i=0; i<rx_buffer_length; i++) {
            // push through half-band decimator
            std::complex<float>decim_out;
            resamp2_crcf_decim_execute(decim, &data_rx[2*i], &decim_out);

            // run through ofdm frame synchronizer
            ofdmframesync_execute(fs, &decim_out, 1);
        }
#endif
    }
 
    // stop data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    printf("\n");
    printf("usrp data transfer complete\n");

 
    // print results
    float data_rate = userdata.num_valid_bytes_received * 8.0f / num_seconds;
    float percent_valid = (userdata.num_packets_received == 0) ?
                          0.0f :
                          100.0f * (float)userdata.num_valid_packets_received / (float)userdata.num_packets_received;
    printf("    packets received    : %6u\n", userdata.num_packets_received);
    printf("    valid packets       : %6u (%6.2f%%)\n", userdata.num_valid_packets_received,percent_valid);
    printf("    data rate           : %8.4f kbps\n", data_rate*1e-3f);

    // destroy objects
    resamp_crcf_destroy(resamp);
    resamp2_crcf_destroy(decim);
    bpacketsync_destroy(ps);
    modem_destroy(demod);
    ofdmframesync_destroy(fs);

    return 0;
}

