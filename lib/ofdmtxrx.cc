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
#include <liquid/liquid.h>

#include "ofdmtxrx.h"

// default constructor
//  _M              :   OFDM: number of subcarriers
//  _cp_len         :   OFDM: cyclic prefix length
//  _taper_len      :   OFDM: taper prefix length
ofdmtxrx::ofdmtxrx(unsigned int _M,
                   unsigned int _cp_len,
                   unsigned int _taper_len)
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
    framegen = ofdmflexframegen_create(M, cp_len, taper_len, p, &fgprops);
    fgbuffer_len = M + cp_len;
    fgbuffer = (std::complex<float>*) malloc(fgbuffer_len * sizeof(std::complex<float>));
    
    // create frame synchronizer
    framesync = ofdmflexframesync_create(M, cp_len, taper_len, p, ofdmtxrx_callback, (void*)this);
    // TODO: create buffer

    // create usrp objects
    uhd::device_addr_t dev_addr;
    usrp_tx = uhd::usrp::multi_usrp::make(dev_addr);
    usrp_rx = uhd::usrp::multi_usrp::make(dev_addr);
    usrp_rx->set_rx_freq(462e6);
    usrp_rx->set_rx_rate(400e3);
    usrp_rx->set_rx_antenna("RX2");

    // reset transceiver
    Reset();
}

// destructor
ofdmtxrx::~ofdmtxrx()
{
    // destroy framing objects
    ofdmflexframegen_destroy(framegen);
    ofdmflexframesync_destroy(framesync);

    // free other allocated arrays
    free(fgbuffer);
}

// reset
void ofdmtxrx::Reset()
{
    // reset all objects
    ofdmflexframegen_reset(framegen);
    ofdmflexframesync_reset(framesync);
}

// set properties
void ofdmtxrx::SetFrequency(double _frequency)
{
}

void ofdmtxrx::SetSampleRate(double _sample_rate)
{
}

void ofdmtxrx::SetHardwareGainTx(double _uhd_gain_tx)
{
}

void ofdmtxrx::SetHardwareGainRx(double _uhd_gain_rx)
{
}

void ofdmtxrx::SetSoftwareGainTx(double _gain_tx)
{
}

void ofdmtxrx::SetSoftwareGainRx(double _gain_rx)
{
}

// start transmitter
void ofdmtxrx::StartTransmitter()
{
}

// stop transmitter
void ofdmtxrx::StopTransmitter()
{
}

// start receiver
void ofdmtxrx::StartReceiver()
{
    usrp_rx->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    printf("usrp rx start\n");
}

// stop receiver
void ofdmtxrx::StopReceiver()
{
    usrp_rx->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    printf("usrp rx stop\n");
}

// update payload data on a particular channel
void ofdmtxrx::TransmitPacket(unsigned char * _header,
                              unsigned char * _payload,
                              unsigned int    _payload_len,
                              int             _mod,
                              int             _fec0,
                              int             _fec1)
                              // frame generator properties...
{
}

// receive packet with timeout
//  _timeout        :   timeout (seconds)
//  _header
//  ...
bool ofdmtxrx::ReceivePacket(double             _timeout,
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
// private methods
//
        
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

