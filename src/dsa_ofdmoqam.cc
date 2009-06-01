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
 
// dsa_ofdmoqam.cc
//
// dynamic spectrum access via OFDM/OQAM
 
#include <math.h>
#include <iostream>
#include <complex>
#include <liquid/liquid.h>

#include "usrp_standard.h"
#include "usrp_prims.h"
#include "usrp_dbid.h"
#include "usrp_bytesex.h"
#include "flex.h"
 
#define DEBUG 0

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
    // ofdm/oqam options
    unsigned int num_subcarriers=128;   // number of ofdm/oqam channels
    unsigned int m=6;                   // filter delay
    float beta=0.90f;                   // excess bandwidth factor
    modulation_scheme ms=MOD_QAM;   // modulation scheme
    unsigned int bps=2;             // modulation depth
    float eta=1e-2;                // DSA sensitivity
    float gamma=1.0f;               // DSA threshold

    gamma *= num_subcarriers / 32;
    usrp_standard_tx * utx;
    usrp_standard_rx * urx;
    int tx_db_id;
    int rx_db_id;
    db_base * txdb;
    db_base * rxdb;

    int    nunderruns = 0;
    bool   underrun;
    bool overrun;
    int num_overruns=0;
    //int    total_writes = 100000;
    unsigned int    i;

    // USRP buffer
    //int    buf[SAMPLES_PER_READ];
    //int    bufsize = SAMPLES_PER_READ*4; // Should be multiple of 512 Bytes
    const unsigned int tx_buf_len = 512; // ensure multiple of num_subcarriers
    short tx_buf[tx_buf_len];
    const unsigned int rx_buf_len = 512;
    short rx_buf[rx_buf_len];

    int    interp_rate = 64;
    int    decim_rate = interp_rate/2;            // 8 -> 32 MB/sec
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
    utx->set_nchannels(1);
    urx->set_nchannels(1);
 
    // Set ADC PGA gain
    urx->set_pga(0,0);         // adc pga gain
    urx->set_mux(0x32103210);  // Board A only
 
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

    // TX: set the daughterboard frequency
    float fmin, fmax, fstep;
    txdb->get_freq_range(fmin,fmax,fstep);
    printf("fmin/fmax/fstep: %f/%f/%f\n", fmin,fmax,fstep);
    float frequency = 462.5e6;
//    float frequency = 485.0e6f;
    float db_lo_offset = -8e6;
    float db_lo_freq = 0.0f;
    float db_lo_freq_set = frequency + db_lo_offset;
    txdb->set_db_freq(db_lo_freq_set, db_lo_freq);
    printf("lo frequency: %f MHz (actual: %f MHz)\n", db_lo_freq_set/1e6, db_lo_freq/1e6);
    float ddc_freq_set = frequency - db_lo_freq;
    utx->set_tx_freq(USRP_CHANNEL, ddc_freq_set);
    float ddc_freq = utx->tx_freq(USRP_CHANNEL);
    printf("ddc freq: %f MHz (actual %f MHz)\n", ddc_freq_set/1e6, ddc_freq/1e6);


    // RX: set the daughterboard frequency
    rxdb->get_freq_range(fmin,fmax,fstep);
    printf("fmin/fmax/fstep: %f/%f/%f\n", fmin,fmax,fstep);
    db_lo_offset = -8e6;
    db_lo_freq = 0.0f;
    db_lo_freq_set = frequency + db_lo_offset;
    rxdb->set_db_freq(db_lo_freq_set, db_lo_freq);
    printf("(rx) lo frequency: %f MHz (actual: %f MHz)\n", db_lo_freq_set/1e6, db_lo_freq/1e6);
    ddc_freq_set = frequency - db_lo_freq;
    urx->set_rx_freq(USRP_CHANNEL, ddc_freq_set);
    ddc_freq = urx->rx_freq(USRP_CHANNEL);
    printf("(rx) ddc freq: %f MHz (actual %f MHz)\n", ddc_freq_set/1e6, ddc_freq/1e6);

    // enable automatic transmit/receive (sets receive RF chain to "TX/RF" port on USRP)
    txdb->set_auto_tr(true);
    rxdb->set_auto_tr(true);

    // create channelizer
    unsigned int k=num_subcarriers;
    ofdmoqam cs = ofdmoqam_create(k, m, beta, 0.0f, OFDMOQAM_SYNTHESIZER);
    ofdmoqam ca = ofdmoqam_create(k, m, beta, 0.0f, OFDMOQAM_ANALYZER);
    unsigned int k0 = 0.2*k;    // lo guard
    unsigned int k1 = 0.8*k;    // hi guard

    // set channelizer gain
    float gain[k];
    unsigned int ki;
#if 0
    for (i=0; i<k; i++) {
        ki = (i + k/2) % k;
        gain[i] = (ki<k0) || (ki>k1) ? 0.0f : 1.0f;
        //printf("%3u >> %3u : %3u : %3u : %f\n", i, k0, ki, k1, gain[i]);
        //gain[i] = 1.0f;
    }
#endif

    std::complex<float> x[k];
    std::complex<float> X[k];
    std::complex<float> y;
    std::complex<float> buffer[rx_buf_len/2];
    float sensing[k];
    float sensing_warped[k];
    for (i=0; i<k; i++)
        sensing[i] = 0.0f;

    modem linmod = modem_create(ms, bps);
    unsigned int s;

    // generate data buffer
    short I, Q;
    txdb->set_enable(true);

    unsigned int t;

    unsigned int j, n;
    // Do USRP Samples Reading 

    while (true) {

        // start receiver
        //usleep(10e3);
        
#if DEBUG
        FILE * fid = fopen("dsa_rx.m","w");
        t = 0;
        fprintf(fid,"%% auto-generated file\n\n");
        fprintf(fid,"x = [];\n");
#endif
        float rms = 0.0f;

        urx->start();
        //printf("receiver enabled\n");
        // clear channelizer internal filter state
        //ofdmoqam_clear(ca);

        // clear sensing data
        for (i=0; i<k; i++) sensing[i] = 1.0f;
        for (i=0; i<200; i++) {
            urx->read(rx_buf, rx_buf_len*sizeof(short), &overrun); 

            // convert to float
            std::complex<float> sample;
            for (n=0; n<rx_buf_len; n+=2) {
                sample.real() = (float) ( rx_buf[n+0]) * 0.01f;
                sample.imag() = (float) (-rx_buf[n+1]) * 0.01f;

                buffer[n/2] = sample;
#if DEBUG
                fprintf(fid,"x(%4u) = %12.4e + j*%12.4e;\n", t++, sample.real(), sample.imag());
#endif
                rms += abs(sample);
            }

            // run analysis
            for (j=0; j<rx_buf_len/2; j+=k) {
                memmove(x,&buffer[j],k*sizeof(std::complex<float>));
                ofdmoqam_execute(ca,buffer,X);

                // if (t < m) continue;
                // else       t++;
                for (n=0; n<k; n++) {
                    sensing[n] *= 1-eta;
                    sensing[n] += eta * abs(X[n]);
                }
            }


        }
        urx->stop();
        //printf("receiver disabled\n");
        
        // compute warped sensing vector
        for (n=0; n<k; n++) {

            unsigned int wp = (n+1)   % k;
            unsigned int wn = (n-1+k) % k;
            sensing_warped[n] = sensing[n] + 
                0.15*(sensing[wp] + sensing[wn]);
            sensing_warped[n] /= gamma;

            // set gain
            gain[n] = (sensing_warped[n] < 1.0f) ? 1.0f : 0.0f;
            ki = (n+k/2)%k;
            if ( (ki<k0) || (ki>k1) )
                gain[n] = 0.0f;
        }
            
        // print results
        printf("----------\n");
        for (n=0; n<k; n++) {
            ki = (n+k/2)%k;
            printf("%3u : %12.8f %c\n", ki, sensing_warped[ki], (gain[ki]>0.5f) ? '*' : ' ');
        }

        printf("rms : %12.8f dB\n", 20*log10(rms));

#if DEBUG
        fclose(fid);
        printf("results written to dsa_rx.m\n");
        exit(0);
#endif


#if 1
        //printf("transmitter enabled\n");
        utx->start();        // Start data transfer

        // flush ofdmoqam buffers
        for (i=0; i<k; i++)
            X[i] = 0.0f;
        for (i=0; i<2*m; i++)
            ofdmoqam_execute(cs,X,x);

        for (i = 0; i < 2500; i++) {
        //while (true) {
            t=0;

            // generate data for USRP buffer
            for (j=0; j<tx_buf_len; j+=2*k) {

                // generate frame data
                for (n=0; n<k; n++) {
                    s = modem_gen_rand_sym(linmod);
                    modem_modulate(linmod, s, &y);

                    X[n] = y*gain[n];
                }

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
        //printf("transmitter disabled\n");
#endif
    } // while (true)

    // clean it up
    ofdmoqam_destroy(cs);
    ofdmoqam_destroy(ca);
    modem_destroy(linmod);
    delete utx;
    return 0;
}

