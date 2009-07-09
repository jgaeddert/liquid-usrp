 // Simple C++ USRP interfacing demonstration program
 //
 //
 // This program was derived and modified from test_usrp_standard_rx.cc 
 
 /* -*- c++ -*- */
 /*
  * Copyright 2003,2006,2007,2008 Free Software Foundation, Inc.
  * 
  * This file is part of GNU Radio
  * 
  * GNU Radio is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 3, or (at your option)
  * any later version.
  * 
  * GNU Radio is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  * 
  * You should have received a copy of the GNU General Public License
  * along with GNU Radio; see the file COPYING.  If not, write to
  * the Free Software Foundation, Inc., 51 Franklin Street,
  * Boston, MA 02110-1301, USA.
  */
 
 
#include <math.h>
#include <iostream>
#include <complex>
#include <liquid/liquid.h>

#include "usrp_standard.h"
#include "usrp_prims.h"
#include "usrp_dbid.h"
#include "flex.h"
 
/*
 SAMPLES_PER_READ :Each sample is consists of 4 bytes (2 bytes for I and 
 2 bytes for Q. Since the reading length from USRP should be multiple of 512 
 bytes see "usrp_basic.h", then we have to read multiple of 128 samples each 
 time (4 bytes * 128 sample = 512 bytes)  
 */
#define SAMPLES_PER_READ    (512)       // Must be a multiple of 128
#define USRP_CHANNEL        (0)
 
static int callback(unsigned char * _header,  int _header_valid,
                    unsigned char * _payload, int _payload_valid,
                    void * _userdata)
{
    std::cout << "********* callback invoked, ";// << std::endl;
    if ( !_header_valid ) {
        printf("header crc : FAIL\n");
    } else if ( !_payload_valid ) {
        printf("payload crc : FAIL\n");
    } else {
        printf("packet id: %u\n", (unsigned int ) _header[0]);
    }
    return 0;
}

void usage() {
    printf("packet_tx:\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  b     :   bandwidth [Hz]\n");
    printf("  t     :   run time [seconds]\n");
    printf("  q     :   quiet\n");
    printf("  v     :   verbose\n");
    printf("  u,h   :   usage/help\n");
}


int main (int argc, char **argv)
{
    bool   loopback_p = false;
    bool   counting_p = false;
    bool   width_8_p = false;
    int    which_board = 0;
    //int    decim = 256;            // 8 -> 32 MB/sec
    double center_freq = 0;
    int    fusb_block_size = 0;
    int    fusb_nblocks = 0;
    int    nchannels = 1;
    int    gain = 0;
    int    mode = 0;
    int    noverruns = 0;
    bool   overrun;
    int    total_reads = 100000;
    int    i;
    const int    rx_buf_len = 512*2; // Should be multiple of 512 Bytes
    short  rx_buf[rx_buf_len];
 

    // command-line options
    bool verbose = true;

    float min_bandwidth = (32e6 / 512.0);
    float max_bandwidth = (32e6 /   4.0);

    float frequency = 462.0e6;
    float bandwidth = min_bandwidth;
    float num_seconds = 5.0f;

    unsigned int packet_spacing=8;
 
    //
    int d;
    while ((d = getopt(argc,argv,"f:b:t:qvuh")) != EOF) {
        switch (d) {
        case 'f':
            frequency = atof(optarg);
            break;
        case 'b':
            bandwidth = atof(optarg);
            break;
        case 't':
            num_seconds = atof(optarg);
            break;
        case 'q':
            verbose = false;
            break;
        case 'v':
            verbose = true;
            break;
        case 'u':
        case 'h':
        default:
            usage();
            return 0;
        }
    }

    // compute interpolation rate
    unsigned int decim_rate = (unsigned int)(16e6 / bandwidth);
    
    // ensure multiple of 2
    decim_rate = (decim_rate >> 1) << 1;

    // update actual bandwidth
    bandwidth = 16e6f / (float)(decim_rate);

    if (bandwidth > max_bandwidth) {
        printf("error: maximum bandwidth exceeded (%8.4f MHz)\n", max_bandwidth*1e-6);
        return 0;
    } else if (bandwidth < min_bandwidth) {
        printf("error: minimum bandwidth exceeded (%8.4f kHz)\n", min_bandwidth*1e-3);
        return 0;
    } if (packet_spacing < 1) {
        printf("error: packet spacing must be greater than 0\n");
        return 0;
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));


    if (loopback_p)    mode |= usrp_standard_rx::FPGA_MODE_LOOPBACK;
   
    if (counting_p)    mode |= usrp_standard_rx::FPGA_MODE_COUNTING;
 
 
 
    usrp_standard_rx *urx =  usrp_standard_rx::make (which_board, decim_rate, 1, -1, mode,
                fusb_block_size, fusb_nblocks);
 
    if (urx == 0) {
        fprintf (stderr, "Error: usrp_standard_rx::make\n");
        exit (1);
    }
 
    if (width_8_p) {
        int width = 8;
        int shift = 8;
        bool want_q = true;
        if (!urx->set_format(usrp_standard_rx::make_format(width, shift, want_q))) {
            fprintf (stderr, "Error: urx->set_format\n");
            exit (1);
       }
    }

    // daughterboard
    int rx_db0 = urx->daughterboard_id(0);
    db_base * rx_db0_control;   // from ossie
    std::cout << "rx db slot 0 : " << usrp_dbid_to_string(rx_db0) << std::endl;
 
    // Set DDC center frequency
    urx->set_rx_freq (0, center_freq);
 
     // Set Number of channels
    urx->set_nchannels(nchannels);
 
    // Set ADC PGA gain
    urx->set_pga(0,gain);
 
    // Set FPGA Mux
    //urx->set_mux(0x32103210); // Board A only
 
    // Set DDC decimation rate
    urx->set_decim_rate(decim_rate);
  
    // Set DDC phase 
    urx->set_ddc_phase(0,0);

    if (rx_db0 == USRP_DBID_FLEX_400_RX_MIMO_B) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400_RX_MIMO_B\n");
        rx_db0_control = new db_flex400_rx_mimo_b(urx,0);
    } else if (rx_db0 == USRP_DBID_FLEX_400_RX) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400_RX\n");
        rx_db0_control = new db_flex400_rx(urx,0);
    } else {
        printf("use usrp db flex 400 rx MIMO B\n");
        return 0;
    }

    rx_db0_control->set_auto_tr(true);

    //
    // USRP TX
    //
    usrp_standard_tx *utx = usrp_standard_tx::make(0, 512);
    if (utx == 0) {
        fprintf (stderr, "Error: usrp_standard_tx::make\n");
        exit (1);
    }
    utx->set_nchannels(1);
    // daughterboard
    int tx_db0 = utx->daughterboard_id(0);
    db_base * tx_db0_control;   // from ossie
    std::cout << "tx db slot 0 : " << usrp_dbid_to_string(tx_db0) << std::endl;
    if (tx_db0 == USRP_DBID_FLEX_400_TX_MIMO_B) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400_TX_MIMO_B\n");
        tx_db0_control = new db_flex400_tx_mimo_b(utx,0);
	} else if (tx_db0 == USRP_DBID_FLEX_400_TX) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400\n");
        tx_db0_control = new db_flex400_tx(utx,0);
    } else {
        printf("use usrp db flex 400 tx MIMO B\n");
        return 0;
    }   
    tx_db0_control->set_auto_tr(true);

    // set the ddc frequency
    urx->set_rx_freq(USRP_CHANNEL, 0.0);

    // set the daughterboard frequency
    //float frequency = 462e6;
    float db_lo_offset = -8e6;
    float db_lo_freq = 0.0f;
    float db_lo_freq_set = frequency + db_lo_offset;
    rx_db0_control->set_db_freq(db_lo_freq_set, db_lo_freq);
    float ddc_freq = frequency - db_lo_freq;
    urx->set_rx_freq(USRP_CHANNEL, ddc_freq);

    // framing
    unsigned int m=3;
    float beta=0.7f;
    framesync64 framesync = framesync64_create(m,beta,callback,NULL);

    // create decimator
    resamp2_crcf decimator = resamp2_crcf_create(37);
    std::complex<float> buffer[rx_buf_len/2];
    std::complex<float> decim_out[rx_buf_len/4];
 
    urx->start();        // Start data transfer
 
    printf("USRP Transfer Started\n");
 
    int n;
    // Do USRP Samples Reading 
    for (i = 0; i < total_reads; i++) {
        urx->read(rx_buf, rx_buf_len*sizeof(short), &overrun); 
            
        if (overrun) {
            printf ("USRP Rx Overrun\n");
            noverruns++;
        }
 
        // Do whatever you want with the data
        //process_data(rx_buf,rx_buf_len);

        // convert to float
        std::complex<float> sample;
        for (n=0; n<rx_buf_len; n+=2) {
            sample.real() = (float) ( rx_buf[n+0]) * 0.01f;
            sample.imag() = (float) (-rx_buf[n+1]) * 0.01f;

            buffer[n/2] = sample;
        }

        // run decimator
        for (n=0; n<rx_buf_len/2; n+=2) {
            resamp2_crcf_decim_execute(decimator, &buffer[n], &decim_out[n/2]);
        }

        /*
        for (n=0; n<rx_buf_len/4; n++)
            printf("%8.4f ", abs(decim_out[n]));
        printf("\n");
        */

        // run through frame synchronizer
        framesync64_execute(framesync, decim_out, rx_buf_len/4);
    }
 
 
    urx->stop();  // Stop data transfer
    printf("USRP Transfer Stopped\n");

    // clean it up
    framesync64_destroy(framesync);
    resamp2_crcf_destroy(decimator);

    delete urx;
    return 0;
}

