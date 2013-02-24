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

//
// ofdmtxrx.cc
//

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <complex>
#include <liquid/liquid.h>

#include "ofdmtxrx.h"

// default constructor
//  _M              :   OFDM: number of subcarriers
//  _cp_len         :   OFDM: cyclic prefix length
//  _taper_len      :   OFDM: taper prefix length
//  _callback       :   frame synchronizer callback function
//  _userdata       :   user-defined data structure
ofdmtxrx::ofdmtxrx(unsigned int       _M,
                   unsigned int       _cp_len,
                   unsigned int       _taper_len,
                   framesync_callback _callback,
                   void *             _userdata)
{
    // validate input
    if (_M < 8) {
        fprintf(stderr,"error: ofdmtxrx::ofdmtxrx(), number of subcarriers must be at least 8\n");
        throw 0;
    } else if (_cp_len < 1) {
        fprintf(stderr,"error: ofdmtxrx::ofdmtxrx(), cyclic prefix length must be at least 1\n");
        throw 0;
    } else if (_taper_len > _cp_len) {
        fprintf(stderr,"error: ofdmtxrx::ofdmtxrx(), taper length cannot exceed cyclic prefix length\n");
        throw 0;
    }

    // set internal properties
    M            = _M;
    cp_len       = _cp_len;
    taper_len    = _taper_len;

    // create frame generator
    unsigned char * p = NULL;   // subcarrier allocation (default)
    ofdmflexframegenprops_init_default(&fgprops);
    fgprops.check           = LIQUID_CRC_32;
    fgprops.fec0            = LIQUID_FEC_NONE;
    fgprops.fec1            = LIQUID_FEC_HAMMING128;
    fgprops.mod_scheme      = LIQUID_MODEM_QPSK;
    fg = ofdmflexframegen_create(M, cp_len, taper_len, p, &fgprops);

    // allocate memory for frame generator output (single OFDM symbol)
    fgbuffer_len = M + cp_len;
    fgbuffer = (std::complex<float>*) malloc(fgbuffer_len * sizeof(std::complex<float>));
    
    // create frame synchronizer
    fs = ofdmflexframesync_create(M, cp_len, taper_len, p, _callback, _userdata);
    // TODO: create buffer

    // create usrp objects
    uhd::device_addr_t dev_addr;
    usrp_tx = uhd::usrp::multi_usrp::make(dev_addr);
    usrp_rx = uhd::usrp::multi_usrp::make(dev_addr);

    // initialize default tx values
    set_tx_freq(462.0e6f);
    set_tx_rate(1200e3);
    set_tx_gain_soft(-12.0f);
    set_tx_gain_uhd(40.0f);

    // initialize default rx values
    set_rx_freq(462.0e6f);
    set_rx_rate(1200e3);
    set_rx_gain_uhd(20.0f);

    // reset transceiver
    reset_tx();
    reset_rx();

    // create and start rx thread
    pthread_create(&rx_process, NULL, ofdmtxrx_rx_worker, (void*)this);
    
    // create and start tx thread
}

// destructor
ofdmtxrx::~ofdmtxrx()
{
    printf("waiting for process to finish...\n");
    void * exit_status;
    pthread_join(rx_process, &exit_status);

    // destroy framing objects
    ofdmflexframegen_destroy(fg);
    ofdmflexframesync_destroy(fs);

    // free other allocated arrays
    free(fgbuffer);
    
    // TODO: output debugging file
    //ofdmflexframesync_debug_print(fs, "ofdmtxrx_debug.m");
}


// 
// transmitter methods
//

// set transmitter frequency
void ofdmtxrx::set_tx_freq(float _tx_freq)
{
    usrp_tx->set_tx_freq(_tx_freq);
}

// set transmitter sample rate
void ofdmtxrx::set_tx_rate(float _tx_rate)
{
    usrp_tx->set_tx_rate(_tx_rate);
}

// set transmitter software gain
void ofdmtxrx::set_tx_gain_soft(float _tx_gain_soft)
{
    tx_gain = powf(10.0f, _tx_gain_soft/20.0f);
}

// set transmitter hardware (UHD) gain
void ofdmtxrx::set_tx_gain_uhd(float _tx_gain_uhd)
{
    usrp_tx->set_tx_gain(_tx_gain_uhd);
}

// set transmitter antenna
void ofdmtxrx::set_tx_antenna(char * _tx_antenna)
{
    usrp_rx->set_tx_antenna(_tx_antenna);
}

// reset transmitter objects and buffers
void ofdmtxrx::reset_tx()
{
    ofdmflexframegen_reset(fg);
}

// start transmitter stream
void ofdmtxrx::start_tx()
{
    // set up the metadta flags
    metadata_tx.start_of_burst = false; // never SOB when continuous
    metadata_tx.end_of_burst   = false; // 
    metadata_tx.has_time_spec  = false; // set to false to send immediately
}

// stop transmitter stream
void ofdmtxrx::stop_tx()
{
    //TODO: flush buffers

    // send a mini EOB packet
    metadata_tx.start_of_burst = false;
    metadata_tx.end_of_burst   = true;

    usrp_tx->get_device()->send("", 0, metadata_tx,
        uhd::io_type_t::COMPLEX_FLOAT32,
        uhd::device::SEND_MODE_FULL_BUFF
    );

    // reset tx objects
    reset_tx();
}

// update payload data on a particular channel
void ofdmtxrx::transmit_packet(unsigned char * _header,
                               unsigned char * _payload,
                               unsigned int    _payload_len,
                               int             _mod,
                               int             _fec0,
                               int             _fec1)
{
    // fector buffer to send data to device
    std::vector<std::complex<float> > usrp_buffer(fgbuffer_len);

    // set properties
    fgprops.mod_scheme  = _mod;
    fgprops.fec0        = _fec0;
    fgprops.fec1        = _fec1;
    ofdmflexframegen_setprops(fg, &fgprops);

    // assemble frame
    ofdmflexframegen_assemble(fg, _header, _payload, _payload_len);

    // generate a single OFDM frame
    int last_symbol=0;
    while (!last_symbol) {

        // generate symbol
        last_symbol = ofdmflexframegen_writesymbol(fg, fgbuffer);

        // copy symbol and apply gain
        unsigned int i;
        for (i=0; i<fgbuffer_len; i++)
            usrp_buffer[i] = fgbuffer[i] * tx_gain;

        // send samples to the device
        usrp_tx->get_device()->send(
            &usrp_buffer.front(), usrp_buffer.size(),
            metadata_tx,
            uhd::io_type_t::COMPLEX_FLOAT32,
            uhd::device::SEND_MODE_FULL_BUFF
        );

    } // while loop
}

// 
// receiver methods
//

// set receiver frequency
void ofdmtxrx::set_rx_freq(float _rx_freq)
{
    usrp_rx->set_rx_freq(_rx_freq);
}

// set receiver sample rate
void ofdmtxrx::set_rx_rate(float _rx_rate)
{
    usrp_rx->set_rx_rate(_rx_rate);
}

// set receiver software gain
void ofdmtxrx::set_rx_gain_soft(float _rx_gain_soft)
{
    // nothing to do
}

// set receiver hardware (UHD) gain
void ofdmtxrx::set_rx_gain_uhd(float _rx_gain_uhd)
{
    usrp_rx->set_rx_gain(_rx_gain_uhd);
}

// set receiver antenna
void ofdmtxrx::set_rx_antenna(char * _rx_antenna)
{
    usrp_rx->set_rx_antenna(_rx_antenna);
}

// reset receiver objects and buffers
void ofdmtxrx::reset_rx()
{
    ofdmflexframesync_reset(fs);
}

// start receiver
void ofdmtxrx::start_rx()
{
    usrp_rx->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    printf("usrp rx start\n");
}

// stop receiver
void ofdmtxrx::stop_rx()
{
    usrp_rx->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    printf("usrp rx stop\n");
}

// receive packet with timeout
//  _timeout        :   timeout (seconds)
//  _header
//  ...
bool ofdmtxrx::receive_packet(float              _timeout,
                              unsigned char **   _header,
                              int  *             _header_valid,
                              unsigned char **   _payload,
                              unsigned int  *    _payload_len,
                              int  *             _payload_valid,
                              framesyncstats_s * _stats)
{
    usleep( 1 + ceil(_timeout*1e6) );
    return false;
}

//
// additional methods
//

// enable debugging
void ofdmtxrx::debug_enable()
{
    ofdmflexframesync_debug_enable(fs);
}

// disable debugging
void ofdmtxrx::debug_disable()
{
    ofdmflexframesync_debug_disable(fs);
}

//
// private methods
//

// receiver worker thread
void * ofdmtxrx_rx_worker(void * _arg)
{
    // type cast input argument as ofdmtxrx object
    ofdmtxrx * txcvr = (ofdmtxrx*) _arg;

    // sleep...
    printf("rx worker sleeping for 8 seconds...\n");
    usleep(8000000);
    printf("rx worker waking...\n");

    // return
    pthread_exit(NULL);
}

#if 0
// callback function
int ofdmtxrx_callback(unsigned char *  _header,
                      int              _header_valid,
                      unsigned char *  _payload,
                      unsigned int     _payload_len,
                      int              _payload_valid,
                      framesyncstats_s _stats,
                      void *           _userdata)
{
    // type case pointer
    //ofdmtxrx * txcvr = (ofdmtxrx*) _userdata;
    
    printf("***** rssi=%7.2fdB evm=%7.2fdB, header[%4s], payload[%4s]",
            _stats.rssi, _stats.evm,
            _header_valid  ? "pass" : "FAIL",
            _payload_valid ? "pass" : "FAIL");

    return 0;
}
#endif
