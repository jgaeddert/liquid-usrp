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
 
// Dummy Function to process USRP data
void process_data(int *buffer)
{
/*
Each buffer element, for example buffer[0] contains 4 bytes 
2 bytes For (I) and 2 bytes for (Q). 
*/
 
    unsigned int i;
    short I,Q;
    float e=0.0f;
    for (i=0; i<SAMPLES_PER_READ; i++) {
        I =  buffer[i] & 0x0000ffff;
        Q = (buffer[i] & 0xffff0000) >> 16;
        //printf("(%5d+j%5d), ", I, Q);
        e += fabsf(I) + fabsf(Q);
    }
    //printf("\n");
    e /= SAMPLES_PER_READ;
    printf("e: %8.4f\n", e);

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
    int    total_writes = 100000;
    int    i;
    //int    buf[SAMPLES_PER_READ];
    //int    bufsize = SAMPLES_PER_READ*4; // Should be multiple of 512 Bytes
    const int tx_buf_len = 512;
    short tx_buf[tx_buf_len];

#if 0 
    if (loopback_p)    mode |= usrp_standard_tx::FPGA_MODE_LOOPBACK;
   
    if (counting_p)    mode |= usrp_standard_tx::FPGA_MODE_COUNTING;
#endif
 
#if 1
    int    which_board = 0;
    //int    decim = 256;            // 8 -> 32 MB/sec
    int    interp_rate = 512;
    int    fusb_block_size = 0;
    int    fusb_nblocks = 0;
    //int    gain = 0;
    int    mode = 0;
    usrp_standard_tx *utx = 
        usrp_standard_tx::make (which_board, interp_rate, 1, -1, //mode,
                fusb_block_size, fusb_nblocks);
#else
    usrp_standard_tx *utx = usrp_standard_tx::make(0, 512);
#endif
 
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
    float frequency = 462e6;
    float db_lo_offset = -8e6;
    float db_lo_freq = 0.0f;
    float db_lo_freq_set = frequency + db_lo_offset;
    tx_db0_control->set_db_freq(db_lo_freq_set, db_lo_freq);
    printf("lo frequency: %f MHz (actual: %f MHz)\n", db_lo_freq_set/1e6, db_lo_freq/1e6);
    float ddc_freq_set = frequency - db_lo_freq;
    utx->set_tx_freq(USRP_CHANNEL, ddc_freq_set);
    float ddc_freq = utx->tx_freq(USRP_CHANNEL);
    printf("ddc freq: %f MHz (actual %f MHz)\n", ddc_freq_set/1e6, ddc_freq/1e6);

    // create interpolator
    unsigned int k=4; // samples per symbol
    unsigned int m=3; // delay
    float beta=0.3f;  // excess bandwidth factor

    unsigned int h_len = 2*k*m+1;
    float h[h_len];
    design_rrc_filter(k,m,beta,0,h);
    h[h_len-1] = h[0];
    interp_crcf interpolator = interp_crcf_create(k,h,h_len);
    std::complex<float> interp_buffer[k];
    std::complex<float> symbol;

    modulation_scheme ms = MOD_PSK;
    unsigned int bps = 2;
    modem linmod = modem_create(ms, bps);
    unsigned int s;

    // generate data buffer
    short I, Q;
    printf("USRP Transfer Started\n");
    tx_db0_control->set_enable(true);
    utx->start();        // Start data transfer
 
    unsigned int j, n;
    // Do USRP Samples Reading 
    for (i = 0; i < total_writes; i++) {
        // generate random data
        for (j=0; j<tx_buf_len; j+=2*k) {
            symbol.real() = rand()%2 ? 1.0f : -1.0f;
            symbol.imag() = rand()%2 ? 1.0f : -1.0f;

            s = modem_gen_rand_sym(linmod);
            modem_modulate(linmod, s, &symbol);

            // run interpolator
            interp_crcf_execute(interpolator, symbol, interp_buffer);
            for (n=0; n<k; n++) {
                I = (short) (interp_buffer[n].real() * 1000);
                Q = (short) (interp_buffer[n].imag() * 1000);

                tx_buf[j+2*n+0] = host_to_usrp_short(I);
                tx_buf[j+2*n+1] = host_to_usrp_short(Q);
            }
        }

        //utx->write(&buf, bufsize, &underrun); 
        int rc = utx->write(tx_buf, tx_buf_len*sizeof(short), &underrun); 
            
        if (underrun) {
            printf ("USRP tx underrun\n");
            nunderruns++;
        }

        if (rc < 0) {
            printf("error occurred with USRP\n");
            exit(0);
        } else if (rc != tx_buf_len*sizeof(short)) {
            printf("error: did not write proper length\n");
            exit(0);
        }
 
        // Do whatever you want with the data
        //process_data(&buf[0]);
    }
 
 
    utx->stop();  // Stop data transfer
    printf("USRP Transfer Stopped\n");

    // clean it up
    interp_crcf_destroy(interpolator);
    modem_destroy(linmod);
    delete utx;
    return 0;
}

