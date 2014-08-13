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
// multichanneltx.cc
//

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <complex>
#include <vector>
#include <liquid/liquid.h>

#include "multichanneltx.h"

// default constructor
//  _num_channels   :   number of channels
//  _M              :   OFDM: number of subcarriers
//  _cp_len         :   OFDM: cyclic prefix length
//  _taper_len      :   OFDM: taper prefix length
//  _p              :   OFDM: subcarrier allocation
multichanneltx::multichanneltx(unsigned int    _num_channels,
                               unsigned int    _M,
                               unsigned int    _cp_len,
                               unsigned int    _taper_len,
                               unsigned char * _p)
{
    // validate input
    if (_num_channels < 1) {
        fprintf(stderr,"error: multichanneltx::multichanneltx(), must have at least one channel\n");
        throw 0;
    } else if (_M < 8) {
        fprintf(stderr,"error: multichanneltx::multichanneltx(), number of subcarriers must be at least 8\n");
        throw 0;
    } else if (_cp_len < 1) {
        fprintf(stderr,"error: multichanneltx::multichanneltx(), cyclic prefix length must be at least 1\n");
        throw 0;
    } else if (_taper_len > _cp_len) {
        fprintf(stderr,"error: multichanneltx::multichanneltx(), taper length cannot exceed cyclic prefix length\n");
        throw 0;
    }
    unsigned int i;

    // set internal properties
    num_channels = _num_channels;
    M            = _M;
    cp_len       = _cp_len;
    taper_len    = _taper_len;

    // create frame generators
    ofdmflexframegenprops_s fgprops;
    ofdmflexframegenprops_init_default(&fgprops);
    fgprops.check           = LIQUID_CRC_32;
    fgprops.fec0            = LIQUID_FEC_NONE;
    fgprops.fec1            = LIQUID_FEC_HAMMING128;
    fgprops.mod_scheme      = LIQUID_MODEM_QPSK;
    framegen = (ofdmflexframegen*)     malloc(num_channels * sizeof(ofdmflexframegen));
    fgbuffer = (std::complex<float>**) malloc(num_channels * sizeof(std::complex<float>*));
    fgbuffer_len = M + cp_len;
    for (i=0; i<num_channels; i++) {
        framegen[i] = ofdmflexframegen_create(M, cp_len, taper_len, _p, &fgprops);
        fgbuffer[i] = (std::complex<float>*) malloc(fgbuffer_len * sizeof(std::complex<float>));
    }
    
    // design custom filterbank channelizer
    unsigned int m  = 13;       // prototype filter delay
    float As        = 60.0f;    // filter stop-band attenuation
    channelizer = firpfbch_crcf_create_kaiser(LIQUID_SYNTHESIZER, 2*num_channels, m, As);

    // channelizer input/output arrays
    X = (std::complex<float>*) malloc( 2 * num_channels * sizeof(std::complex<float>) );
    x = (std::complex<float>*) malloc( 2 * num_channels * sizeof(std::complex<float>) );

    // create NCO to center spectrum
    float offset = -0.5f*(float)(num_channels-1) / (float)num_channels * M_PI;
    nco = nco_crcf_create(LIQUID_VCO);
    nco_crcf_set_frequency(nco, offset);

    // reset base station transmitter
    Reset();
}

// destructor
multichanneltx::~multichanneltx()
{
    // destroy NCO
    nco_crcf_destroy(nco);

    // destroy channelizer
    firpfbch_crcf_destroy(channelizer);

    // destroy frame generators
    unsigned int i;
    for (i=0; i<num_channels; i++) {
        ofdmflexframegen_destroy(framegen[i]);
        free(fgbuffer[i]);
    }
    free(framegen);
    free(fgbuffer);

    // TODO: free other buffers
    free(X);
    free(x);
}

// reset
void multichanneltx::Reset()
{
    // reset all objects
    unsigned int i;
    for (i=0; i<num_channels; i++)
        ofdmflexframegen_reset(framegen[i]);

    firpfbch_crcf_destroy(channelizer);

    //nco_crcf_reset(nco);
    
    for (i=0; i<2*num_channels; i++) {
        X[i] = 0.0f;
        x[i] = 0.0f;
    }

    // reset read index of buffer to buffer length
    // to indicate we need to generate frame data
    fgbuffer_index = fgbuffer_len;

    // clear frame generator buffers
    for (i=0; i<num_channels; i++)
        memset(fgbuffer[i], 0x00, fgbuffer_len*sizeof(std::complex<float>));
}

// is channel ready for more data?
int multichanneltx::IsChannelReadyForData(unsigned int _channel)
{
    // validate channel id
    if (_channel >= num_channels) {
        fprintf(stderr,"error: multichanneltx:IsChannelReadyForData(%u), invalid channel id\n", _channel);
        throw 0;
    }

    // if it's assembled, then it's not ready yet
    return ofdmflexframegen_is_assembled(framegen[_channel]) ? 0 : 1;
}

// update payload data on a particular channel
void multichanneltx::UpdateData(unsigned int    _channel,
                                unsigned char * _header,
                                unsigned char * _payload,
                                unsigned int    _payload_len,
                                int             _mod,
                                int             _fec0,
                                int             _fec1)
                                // frame generator properties...
{
    // check to see if the channel is indeed waiting for data
    if (_channel >= num_channels) {
        fprintf(stderr,"error: multichanneltx:UpdateData(%u), invalid channel id\n", _channel);
        throw 0;
    } else if (!IsChannelReadyForData(_channel)) {
        fprintf(stderr,"warning: multichanneltx:UpdateData(%u), channel not ready yet\n", _channel);
        return;
    }

    // set frame properties
    ofdmflexframegenprops_s fgprops = {LIQUID_CRC_32, _fec0, _fec1, _mod};
    ofdmflexframegen_setprops(framegen[_channel], &fgprops);

    // assemble frame
    ofdmflexframegen_assemble(framegen[_channel], _header, _payload, _payload_len);
}
            
// Generate samples for transmission
void multichanneltx::GenerateSamples(std::complex<float> * _buffer)
{
    unsigned int i;
   
    // check frame generator buffer index and generate more frame
    // samples if nedessary
    if (fgbuffer_index >= fgbuffer_len) {
        GenerateFrameSamples();
        fgbuffer_index = 0;
    }

    // push through channelizer
    // TODO: load into appropriate filterbank channels
    for (i=0; i<num_channels; i++) {
        // compute appropriate filterbank channel
        //unsigned int m = (2*i + 4*num_channels - num_channels + 1)%(4*num_channels);
        unsigned int m = i;
        X[m] = fgbuffer[i][fgbuffer_index];
    }

    // execute filterbank channelizer as synthesizer
    firpfbch_crcf_synthesizer_execute(channelizer, X, _buffer);

    // center spectrum
#if 0
    nco_crcf_mix_block_up(nco, _buffer, _buffer, 2*num_channels);
#else
    for (i=0; i<2*num_channels; i++) {
        nco_crcf_mix_up(nco, _buffer[i], &_buffer[i]);
        nco_crcf_step(nco);
    }
#endif
    
    // increment frame generator buffer index
    fgbuffer_index++;
}

// generate frame samples from internal frame generator
void multichanneltx::GenerateFrameSamples()
{
    unsigned int i;
    for (i=0; i<num_channels; i++) {
        if ( ofdmflexframegen_is_assembled(framegen[i]) ) {
            // write OFDM frame symbol (ignore return value)
            ofdmflexframegen_writesymbol(framegen[i], fgbuffer[i]);
        } else {
            // not assembled; just produce zeros
            memset(fgbuffer[i], 0x00, fgbuffer_len*sizeof(std::complex<float>));
        }
    }
}

