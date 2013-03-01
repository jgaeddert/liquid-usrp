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
// multichannelrx.h
//

#ifndef __MULTICHANNELRX_H__
#define __MULTICHANNELRX_H__

#include <liquid/liquid.h>

class multichannelrx {
public:
    // default constructor
    //  _num_channels   :   number of channels
    //  _M              :   OFDM: number of subcarriers
    //  _cp_len         :   OFDM: cyclic prefix length
    //  _taper_len      :   OFDM: taper prefix length
    //  _p              :   OFDM: subcarrier allocation
    //  _userdata       :   user-defined data structure array
    //  _callback       :   user-defined callback function
    multichannelrx(unsigned int         _num_channels,
                   unsigned int         _M,
                   unsigned int         _cp_len,
                   unsigned int         _taper_len,
                   unsigned char*       _p,
                   void **              _userdata,
                   framesync_callback * _callback);

    // destructor
    ~multichannelrx();

    // reset multi-channel receiver
    void Reset();

    // accessor methods
    unsigned int GetNumChannels() { return num_channels; }

    // push samples into base station receiver
    void Execute(std::complex<float> * _x,
                 unsigned int          _num_samples);

private:
    // ...
    void RunChannelizer();

    // properties
    unsigned int num_channels;      // number of downlink channels

    // OFDM properties
    unsigned int M;                 // number of subcarriers
    unsigned int cp_len;            // cyclic prefix length
    unsigned int taper_len;         // taper length

    // finite impulse response polyphase filterbank channelizer
    firpfbch_crcf channelizer;      // channelizer size is 2*num_channels
    std::complex<float> * x;        // channelizer input
    std::complex<float> * X;        // channelizer output
    unsigned int buffer_index;      // input index

    // objects
    ofdmflexframesync * framesync;  // array of frame generator objects
    void ** userdata;               // array of userdata pointers
    framesync_callback * callback;  // array of callback functions
    nco_crcf nco;                   // frequency-centering NCO
};

#endif // __MULTICHANNELRX_H__

