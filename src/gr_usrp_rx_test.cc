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
 
 
 #include "usrp_standard.h"           
 
 // Dummy Function to process USRP data
 void process_data(int *buffer)
 {
 /*
 Each buffer element, for example buffer[0] contains 4 bytes 
 2 bytes For (I) and 2 bytes for (Q). 
 */
     /*
     unsigned int i;
     for (i=0; i<4; i++) {
         printf("%6d ", buffer[i]);
     }
     printf("\n");
     */
 }
 
 #define SAMPELS_PER_READ   (512)       // Must be a multiple of 128
 
 /*
 SAMPELS_PER_READ :Each sample is consists of 4 bytes (2 bytes for I and 
 2 bytes for Q. Since the reading length from USRP should be multiple of 512 
 bytes see "usrp_basic.h", then we have to read multiple of 128 samples each 
 time (4 bytes * 128 sample = 512 bytes)  
 */

int main (int argc, char **argv)
{
    bool   loopback_p = false;
    bool   counting_p = false;
    bool   width_8_p = false;
    int    which_board = 0;
    int    decim = 8;            // 8 -> 32 MB/sec
    double center_freq = 0;
    int    fusb_block_size = 0;
    int    fusb_nblocks = 0;
    int    nchannels = 1;
    int    gain = 0;
    int    mode = 0;
    int    noverruns = 0;
    bool   overrun;
    int    total_reads = 10000;
    int    i;
    int    buf[SAMPELS_PER_READ];
    int    bufsize = SAMPELS_PER_READ*4; // Should be multiple of 512 Bytes
     
 
    if (loopback_p)    mode |= usrp_standard_rx::FPGA_MODE_LOOPBACK;
   
    if (counting_p)    mode |= usrp_standard_rx::FPGA_MODE_COUNTING;
 
 
 
    usrp_standard_rx *urx =  usrp_standard_rx::make (which_board, decim, 1, -1, mode,
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
 
    // Set DDC center frequency
    urx->set_rx_freq (0, center_freq);
 
     // Set Number of channels
    urx->set_nchannels(1);
 
    // Set ADC PGA gain
    urx->set_pga(0,gain);
 
    // Set FPGA Mux
    urx->set_mux(0x32103210); // Board A only
 
    // Set DDC decimation rate
    urx->set_decim_rate(decim);
  
    // Set DDC phase 
    urx->set_ddc_phase(0,0);
 
    urx->start();        // Start data transfer
 
    printf("USRP Transfer Started\n");
 
    // Do USRP Samples Reading 
    for (i = 0; i < total_reads; i++) {
        urx->read(&buf, bufsize, &overrun); 
            
        if (overrun) {
            printf ("USRP Rx Overrun\n");
            noverruns++;
        }
 
        // Do whatever you want with the data
        process_data(&buf[0]);
        
    }
 
 
    urx->stop();  // Stop data transfer
    printf("USRP Transfer Stoped\n");
    delete urx;
    return 0;
}

