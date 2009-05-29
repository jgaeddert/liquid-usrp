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
 
// tx_ofdmoqam.cc
//
// transmits OFDM/OQAM signal
 
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
 
int main (int argc, char **argv)
{
    usrp_standard_tx * utx;
    usrp_standard_rx * urx;
    int tx_db_id;
    int rx_db_id;
    db_base * txdb;
    db_base * rxdb;

    //bool   loopback_p = false;
    //bool   counting_p = false;
    //bool   width_8_p = false;
    //double center_freq = 0;
    int    nchannels = 1;
    int    nunderruns = 0;
    bool   underrun;
    //int    total_writes = 100000;
    unsigned int    i;

    // ofdm/oqam options
    unsigned int num_channels=32;   // number of ofdm/oqam channels
    unsigned int m=6;               // filter delay
    modulation_scheme ms=MOD_QAM;   // modulation scheme
    unsigned int bps=2;             // modulation depth

    // USRP buffer
    //int    buf[SAMPLES_PER_READ];
    //int    bufsize = SAMPLES_PER_READ*4; // Should be multiple of 512 Bytes
    const unsigned int tx_buf_len = 512; // ensure multiple of num_channels
    short tx_buf[tx_buf_len];

    int    decim_rate = 256;            // 8 -> 32 MB/sec
    int    interp_rate = 64;
    utx = usrp_standard_tx::make(0, interp_rate);
    urx = usrp_standard_rx::make(0, decim_rate);
 
    if (utx == 0) {
        fprintf (stderr, "Error: usrp_standard_tx::make\n");
        exit (1);
    } else if (urx == 0) {
        fprintf(stderr, "Error: usrp_standard_rx::make\n");
        exit(1);
    }
    
    // tx daughterboard
    tx_db_id = utx->daughterboard_id(0);
    std::cout << "tx db slot 0 : " << usrp_dbid_to_string(tx_db_id) << std::endl;
 
    if (tx_db_id == USRP_DBID_FLEX_400_TX_MIMO_B) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400_TX_MIMO_B\n");
        txdb = new db_flex400_tx_mimo_b(utx,0);
    } else {
        printf("use usrp db flex 400 tx MIMO B\n");
        return 0;
    }   
    txdb->set_enable(true);

    // rx daughterboard
    rx_db_id = urx->daughterboard_id(0);
    std::cout << "rx db slot 0 : " << usrp_dbid_to_string(rx_db_id) << std::endl;
 
    if (rx_db_id == USRP_DBID_FLEX_400_RX_MIMO_B) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400_RX_MIMO_B\n");
        rxdb = new db_flex400_rx_mimo_b(urx,0);
    } else {
        printf("use usrp db flex 400 rx MIMO B\n");
        return 0;
    }   


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



    // set the ddc frequency
    utx->set_tx_freq(USRP_CHANNEL, 0.0);

    // set the daughterboard gain
    float gmin, gmax, gstep;
    txdb->get_gain_range(gmin,gmax,gstep);
    printf("gmin/gmax/gstep: %f/%f/%f\n", gmin,gmax,gstep);
    txdb->set_gain(gmax);

    // set the daughterboard frequency
    float fmin, fmax, fstep;
    txdb->get_freq_range(fmin,fmax,fstep);
    printf("fmin/fmax/fstep: %f/%f/%f\n", fmin,fmax,fstep);
    float frequency = 462.5e6;
    float db_lo_offset = -8e6;
    float db_lo_freq = 0.0f;
    float db_lo_freq_set = frequency + db_lo_offset;
    txdb->set_db_freq(db_lo_freq_set, db_lo_freq);
    printf("lo frequency: %f MHz (actual: %f MHz)\n", db_lo_freq_set/1e6, db_lo_freq/1e6);
    float ddc_freq_set = frequency - db_lo_freq;
    utx->set_tx_freq(USRP_CHANNEL, ddc_freq_set);
    float ddc_freq = utx->tx_freq(USRP_CHANNEL);
    printf("ddc freq: %f MHz (actual %f MHz)\n", ddc_freq_set/1e6, ddc_freq/1e6);

    // create channelizer
    unsigned int k=2*num_channels;
    ofdmoqam cs = ofdmoqam_create(k, m, 0.99f, 0.0f, OFDMOQAM_SYNTHESIZER);

    // set channelizer gain
    float gain[k];
    for (i=0; i<k; i++)
        gain[i] = (i<k/4) || (i>3*k/4) ? 1.0f : 0.0f;
    gain[3] = 0.0f;
    gain[4] = 0.0f;
    gain[5] = 0.0f;
    gain[6] = 0.0f;

    std::complex<float> x[k];
    std::complex<float> X[k];
    std::complex<float> y;

    modem linmod = modem_create(ms, bps);
    unsigned int s;

    // generate data buffer
    short I, Q;
    printf("USRP Transfer Started\n");
    txdb->set_enable(true);
    utx->start();        // Start data transfer

    unsigned int t;

    unsigned int pilot_channel = 3*k/4 - 2;
    unsigned int pilot_symbol = 1;
 
    unsigned int j, n;
    // Do USRP Samples Reading 
    //for (i = 0; i < total_writes; i++) {
    while (true) {
        t=0;

        // generate data for USRP buffer
        for (j=0; j<tx_buf_len; j+=2*k) {

            // generate frame data
            for (n=0; n<k; n++) {
                s = modem_gen_rand_sym(linmod);
                modem_modulate(linmod, s, &y);
                X[n] = y*gain[n];
            }

            // set pilot symbol
            //X[pilot_channel] = pilot_symbol ? 1.0f : -1.0f;
            pilot_symbol = 1-pilot_symbol;

            // execute synthesizer
            ofdmoqam_execute(cs, X, x);

            for (n=0; n<k; n++) {
                I = (short) (x[n].real() * k * 1000);
                Q = (short) (x[n].imag() * k * 1000);

                //printf("%4u : %6d + j%6d\n", t, I, Q);
                //I = 1000;
                //Q = 0;

                tx_buf[t++] = host_to_usrp_short(I);
                tx_buf[t++] = host_to_usrp_short(Q);
            }
        }

        //printf("t : %u (%u)\n", t, tx_buf_len);

#if 0
        // generate random frame data
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
#endif

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
 
    }
 
 
    utx->stop();  // Stop data transfer
    printf("USRP Transfer Stopped\n");

    // clean it up
    ofdmoqam_destroy(cs);
    modem_destroy(linmod);
    delete utx;
    return 0;
}

