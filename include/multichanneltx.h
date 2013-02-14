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
// multichanneltx.h
//

#ifndef __MULTICHANNELTX_H__
#define __MULTICHANNELTX_H__

#include <liquid/liquid.h>

class multichanneltx {
public:
    // default constructor
    //  _num_channels   :   number of channels
    multichanneltx(unsigned int _num_channels);

    // destructor
    ~multichanneltx();

    // reset base station transmitter
    void Reset();

    // accessor methods
    unsigned int GetNumChannels() { return num_channels; }

    // is channel ready for more data?
    int IsChannelReadyForData(unsigned int _channel);

    // update payload data on a particular channel
    void UpdateData(unsigned int    _channel,
                    unsigned char * _header,
                    unsigned char * _payload,
                    unsigned int    _payload_len);
                    // frame generator properties...
            
    // Generate samples for transmission
    void GenerateSamples(std::complex<float> * _buffer);

private:
    // generate frame samples from internal frame generator
    void GenerateFrameSamples();

    // properties
    unsigned int num_channels;      // number of downlink channels

    // finite impulse response polyphase filterbank channelizer
    firpfbch_crcf channelizer;      // channelizer size is 2*num_channels
    std::complex<float> * X;        // channelizer input
    std::complex<float> * x;        // channelizer output

    // objects
    ofdmflexframegen * framegen;    // array of frame generator objects
    std::complex<float> ** fgbuffer;// frame generator output buffers @ M + cp_len
    unsigned int fgbuffer_len;      // length of frame generator buffers
    unsigned int fgbuffer_index;    // read index of buffer
    nco_crcf nco;                   // frequency-centering NCO
    
    //unsigned int * channel_id;      // channelizer IDs
};

#endif // __MULTICHANNELTX_H__

