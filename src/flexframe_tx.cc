 // Simple C++ USRP interfacing demonstration program
 //
 //
 // This program was derived and modified from test_usrp_standard_tx.cc 
 
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
#include <getopt.h>
#include <liquid/liquid.h>

#include "usrp_standard.h"
#include "usrp_prims.h"
#include "usrp_dbid.h"
#include "usrp_bytesex.h"
#include "flex.h"
 
/*
 SAMPLES_PER_READ :Each sample is consists of 4 bytes (2 bytes for I and 
 2 bytes for Q. Since the reading length from USRP should be multiple of 512 
 bytes see "usrp_basic.h", then we have to read multiple of 128 samples each 
 time (4 bytes * 128 sample = 512 bytes)  
 */
#define SAMPLES_PER_READ    (512)       // Must be a multiple of 128
#define USRP_CHANNEL        (0)

void usage() {
    printf("packet_tx:\n");
    printf("  f     :   center frequency [Hz]\n");
    printf("  b     :   bandwidth [Hz]\n");
    printf("  t     :   run time [seconds]\n");
    printf("  n     :   * payload length (bytes)\n");
    printf("  m     :   * mod. scheme: <psk>, dpsk, qam,...\n");
    printf("  p     :   * mod. depth: <1>,2,...8\n");
    printf("  c     :   * fec scheme: <none>,v27,v29,...\n");
    printf("  q     :   quiet\n");
    printf("  v     :   verbose\n");
    printf("  u,h   :   usage/help\n");
}

int main (int argc, char **argv)
{
    //bool   loopback_p = false;
    //bool   counting_p = false;
    //bool   width_8_p = false;
    //double center_freq = 0;
    int    nchannels = 1;
    int    nunderruns = 0;
    bool   underrun;
    int    total_writes = 10000;
    int    i;
    //int    buf[SAMPLES_PER_READ];
    //int    bufsize = SAMPLES_PER_READ*4; // Should be multiple of 512 Bytes

    bool verbose = true;

    float min_bandwidth = (32e6 / 512.0);
    float max_bandwidth = (32e6 /   4.0);

    float frequency = 462.0e6;
    float bandwidth = min_bandwidth;
    float num_seconds = 5.0f;

    unsigned int packet_spacing=2;
    unsigned int payload_len=200;
    fec_scheme fec0 = FEC_NONE;
    fec_scheme fec1 = FEC_HAMMING74;
    modulation_scheme mod_scheme = MOD_QAM;
    unsigned int mod_depth = 4;

#if 0 
    if (loopback_p)    mode |= usrp_standard_tx::FPGA_MODE_LOOPBACK;
   
    if (counting_p)    mode |= usrp_standard_tx::FPGA_MODE_COUNTING;
#endif
 
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
    unsigned int interp_rate = (unsigned int)(32e6 / bandwidth);
    
    // ensure multiple of 4
    interp_rate = (interp_rate >> 2) << 2;

    // update actual bandwidth
    bandwidth = 32e6f / (float)(interp_rate);

    if (bandwidth > max_bandwidth) {
        printf("error: maximum bandwidth exceeded (%8.4f MHz)\n", max_bandwidth*1e-6);
        return 0;
    } else if (bandwidth < min_bandwidth) {
        printf("error: minimum bandwidth exceeded (%8.4f kHz)\n", min_bandwidth*1e-3);
        return 0;
    } else if (packet_spacing < 1) {
        printf("error: packet spacing must be greater than 0\n");
        return 0;
    } else if (payload_len > (1<<16)) {
        printf("error: maximum payload length exceeded: %u > %u\n", payload_len, 1<<16);
        return 0;
    }

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));


    int    which_board = 0;
    //int    decim = 256;            // 8 -> 32 MB/sec
    //int    interp_rate = 512;
    int    fusb_block_size = 0;
    int    fusb_nblocks = 0;
    //int    gain = 0;
    //int    mode = 0;
    usrp_standard_tx *utx = 
        usrp_standard_tx::make (which_board, interp_rate, 1, -1, //mode,
                fusb_block_size, fusb_nblocks);
 
    if (utx == 0) {
        fprintf (stderr, "Error: usrp_standard_tx::make\n");
        exit (1);
    }

#if 0
    if (width_8_p) {
        int width = 8;
        int shift = 8;
        bool want_q = true;
        if (!utx->set_format(usrp_standard_tx::make_format(width, shift, want_q))) {
            fprintf (stderr, "Error: utx->set_format\n");
            exit (1);
       }
    }
#endif

    // daughterboard
    int tx_db0 = utx->daughterboard_id(0);
    db_base * tx_db0_control;   // from ossie
    std::cout << "tx db slot 0 : " << usrp_dbid_to_string(tx_db0) << std::endl;
 
    // Set DDC center frequency
    //utx->set_tx_freq (0, center_freq);
 
     // Set Number of channels
    utx->set_nchannels(nchannels);
 
    // Set ADC PGA gain
    //utx->set_pga(0,gain);
 
    // Set FPGA Mux
    //utx->set_mux(0x32103210); // Board A only
 
    // Set DDC decimation rate
    //utx->set_decim_rate(decim);
  
    // Set DDC phase 
    //utx->set_ddc_phase(0,0);


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

    // set the ddc frequency
    utx->set_tx_freq(USRP_CHANNEL, 0.0);

    // set the daughterboard gain
    float gmin, gmax, gstep;
    tx_db0_control->get_gain_range(gmin,gmax,gstep);
    printf("gmin/gmax/gstep: %f/%f/%f\n", gmin,gmax,gstep);
    tx_db0_control->set_gain(gmax);

    // set the daughterboard frequency
    float fmin, fmax, fstep;
    tx_db0_control->get_freq_range(fmin,fmax,fstep);
    printf("fmin/fmax/fstep: %f/%f/%f\n", fmin,fmax,fstep);
    //float frequency = 462e6;
    float db_lo_offset = -8e6;
    float db_lo_freq = 0.0f;
    float db_lo_freq_set = frequency + db_lo_offset;
    tx_db0_control->set_db_freq(db_lo_freq_set, db_lo_freq);
    printf("lo frequency: %f MHz (actual: %f MHz)\n", db_lo_freq_set/1e6, db_lo_freq/1e6);
    float ddc_freq_set = frequency - db_lo_freq;
    utx->set_tx_freq(USRP_CHANNEL, ddc_freq_set);
    float ddc_freq = utx->tx_freq(USRP_CHANNEL);
    printf("ddc freq: %f MHz (actual %f MHz)\n", ddc_freq_set/1e6, ddc_freq/1e6);

    // packetizer
    packetizer p = packetizer_create(payload_len,fec0,fec1);
    unsigned int packet_len = packetizer_get_packet_length(payload_len,fec0,fec1);
    packetizer_print(p);

    // create flexframegen object
    flexframegenprops_s fgprops;
    fgprops.rampup_len = 64;
    fgprops.phasing_len = 64;
    fgprops.payload_len = packet_len;
    fgprops.mod_scheme = mod_scheme;
    fgprops.mod_bps = mod_depth;
    fgprops.rampdn_len = 64;
    flexframegen fg = flexframegen_create(&fgprops);
    flexframegen_print(fg);


    // framing buffers
    unsigned int frame_len = flexframegen_getframelen(fg);
    div_t dt = div(frame_len,512);
    unsigned int framebuf_len = 512*(dt.quot+1);// + (dt.rem > 0 ? 512 : 0);
    if (framebuf_len < 1024) framebuf_len = 1024;
    std::complex<float> frame[frame_len];
    std::complex<float> mfbuffer[2*framebuf_len];
    std::complex<float> interp_buffer[4*framebuf_len];
    const unsigned int tx_buf_len = 8*framebuf_len;
    short tx_buf[tx_buf_len];

    printf("frame length        :   %u\n", frame_len);
    printf("frame buffer length :   %u\n", framebuf_len);

    // create pulse-shaping interpolator
    unsigned int m=3;
    float beta=0.7f;
    unsigned int h_len = 2*2*m + 1;
    float h[h_len];
    design_rrc_filter(2,m,beta,0,h);
    interp_crcf mfinterp = interp_crcf_create(2,h,h_len);

    // create half-band interpolator
    resamp2_crcf interpolator = resamp2_crcf_create(37,0.0f,60.0f);

    // data buffers
    unsigned char header[8];
    unsigned char payload[payload_len];
    unsigned char packet[packet_len];

    // generate data buffer
    short I, Q;
    printf("USRP Transfer Started\n");
    tx_db0_control->set_enable(true);
    utx->start();        // Start data transfer
 
    unsigned int j, n, pid=0;
    // Do USRP Samples Reading 
    for (i = 0; i < total_writes; i++) {
        // generate the frame / transmit silence
        if ((i%packet_spacing)==0) {
            // generate random data
            // TODO : encode using forward error-correction codec
            for (j=0; j<payload_len; j++)
                payload[j] = rand() % 256;
            // assemble packet
            packetizer_encode(p,payload,packet);
            // write header
            header[0] = (pid >> 8) & 0xff;
            header[1] = (pid     ) & 0xff;
            header[2] = (payload_len >> 8) & 0xff;
            header[3] = (payload_len     ) & 0xff;
            header[4] = (unsigned char)(fec0);
            header[5] = (unsigned char)(fec1);
            if (verbose)
                printf("packet id: %6u\n", pid);
                //printf("packet id: %6u, packet len : %6u\n", pid, packet_len);
            /*
            for (j=0; j<payload_len; j++)
                printf("%.2x ",payload[j]);
            printf("\n");
            for (j=0; j<packet_len; j++)
                printf("%.2x ",packet[j]);
            printf("\n");
            */
            pid = (pid+1)%(1<<16);

            flexframegen_execute(fg, header, packet, frame);

            // interpolate using matched filter
            for (j=0; j<framebuf_len; j++) {
                std::complex<float> x = j<frame_len ? frame[j] : 0.0f;
                interp_crcf_execute(mfinterp, x, &mfbuffer[2*j]);
            }

        } else {
            // flush interpolator with zeros
            for (j=0; j<framebuf_len; j++)
                interp_crcf_execute(mfinterp, 0.0f, &mfbuffer[2*j]);
        }

        // interpolate using half-band interpolator
        for (j=0; j<2*framebuf_len; j++)
            resamp2_crcf_interp_execute(interpolator, mfbuffer[j], &interp_buffer[2*j]);

        for (n=0; n<4*framebuf_len; n++) {
            // prepare data
            I = (short) (interp_buffer[n].real() * 8000);
            Q = (short) (interp_buffer[n].imag() * 8000);

            tx_buf[2*n+0] = host_to_usrp_short(I);
            tx_buf[2*n+1] = host_to_usrp_short(Q);
        }

        // write data

        //utx->write(&buf, bufsize, &underrun); 
        int rc = utx->write(tx_buf, tx_buf_len*sizeof(short), &underrun); 
                    
        if (underrun) {
            printf ("USRP tx underrun\n");
            nunderruns++;
        }

        if (rc < 0) {
            printf("error occurred with USRP\n");
            exit(0);
        } else if (rc != (int)(tx_buf_len*sizeof(short))) {
            printf("error: did not write proper length\n");
            exit(0);
        }
 
    }
 
 
    utx->stop();  // Stop data transfer
    printf("USRP Transfer Stopped\n");

    // clean it up
    packetizer_destroy(p);
    flexframegen_destroy(fg);
    resamp2_crcf_destroy(interpolator);
    interp_crcf_destroy(mfinterp);
    delete utx;
    return 0;
}

