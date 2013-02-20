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
// multichannelrx.cc
//

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <complex>
#include <vector>
#include <liquid/liquid.h>

#include "multichannelrx.h"

#define BST_DEBUG 1

// default constructor
//  _num_channels   :   number of channels
//  _M              :   OFDM: number of subcarriers
//  _cp_len         :   OFDM: cyclic prefix length
//  _taper_len      :   OFDM: taper prefix length
//  _userdata       :   user-defined data structure array
//  _callback       :   user-defined callback function
multichannelrx::multichannelrx(unsigned int         _num_channels,
                               unsigned int         _M,
                               unsigned int         _cp_len,
                               unsigned int         _taper_len,
                               void **              _userdata,
                               framesync_callback * _callback)
{
    // validate input
    if (_num_channels < 1) {
        fprintf(stderr,"error: multichannelrx::multichannelrx(), must have at least one channel\n");
        throw 0;
    } else if (_M < 8) {
        fprintf(stderr,"error: multichannelrx::multichannelrx(), number of subcarriers must be at least 8\n");
        throw 0;
    } else if (_cp_len < 1) {
        fprintf(stderr,"error: multichannelrx::multichannelrx(), cyclic prefix length must be at least 1\n");
        throw 0;
    } else if (_taper_len > _cp_len) {
        fprintf(stderr,"error: multichannelrx::multichannelrx(), taper length cannot exceed cyclic prefix length\n");
        throw 0;
    }
    unsigned int i;

    // set internal properties
    num_channels = _num_channels;
    M            = _M;
    cp_len       = _cp_len;
    taper_len    = _taper_len;

    // create frame generators
    unsigned char * p = NULL;   // subcarrier allocation (default)
    framesync = (ofdmflexframesync*)  malloc(num_channels * sizeof(ofdmflexframesync));
    userdata  = (void **)             malloc(num_channels * sizeof(void *));
    callback  = (framesync_callback*) malloc(num_channels * sizeof(framesync_callback));
    for (i=0; i<num_channels; i++) {
        userdata[i]  = _userdata[i];
        callback[i]  = _callback[i];
        framesync[i] = ofdmflexframesync_create(M, cp_len, taper_len, p, callback[i], userdata[i]);
#if BST_DEBUG
        ofdmflexframesync_debug_enable(framesync[i]);
#endif
    }
    
    // design custom filterbank channelizer
    unsigned int m  = 7;        // prototype filter delay
    float As        = 60.0f;    // stop-band attenuation
    channelizer = firpfbch_crcf_create_kaiser(LIQUID_ANALYZER, 2*num_channels, m, As);

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
multichannelrx::~multichannelrx()
{
    // destroy NCO
    nco_crcf_destroy(nco);

    // destroy channelizer
    firpfbch_crcf_destroy(channelizer);

    // destroy frame synchronizers
    unsigned int i;
    for (i=0; i<num_channels; i++) {
#if BST_DEBUG
        char filename[64];
        sprintf(filename,"framesync_channel%u.m", i);
        ofdmflexframesync_debug_print(framesync[i], filename);
#endif
        ofdmflexframesync_destroy(framesync[i]);
    }
    free(framesync);
    free(userdata);
    free(callback);

    // free other buffers
    free(X);
    free(x);
}

// reset
void multichannelrx::Reset()
{
    // reset all objects
    unsigned int i;
    for (i=0; i<num_channels; i++)
        ofdmflexframesync_reset(framesync[i]);

    firpfbch_crcf_clear(channelizer);

    //nco_crcf_reset(nco);
    
    for (i=0; i<2*num_channels; i++) {
        X[i] = 0.0f;
        x[i] = 0.0f;
    }

    // reset write index of channelizer buffer
    buffer_index = 0;
}

void multichannelrx::Execute(std::complex<float> * _x,
                                  unsigned int          _num_samples)
{
    unsigned int i;
    for (i=0; i<_num_samples; i++) {
#if 1
        // mix signal down and put resulting sample into
        // channelizer input buffer
        nco_crcf_mix_down(nco, _x[i], &x[buffer_index]);
        nco_crcf_step(nco);

        // update buffer index and...
        buffer_index++;
        if (buffer_index == 2*num_channels) {
            // reset index
            buffer_index = 0;

            // run...
            RunChannelizer();
        }
#else
        buffer_index++;
        if ( (buffer_index % (2*num_channels))==0 )
            ofdmflexframesync_execute(framesync[0], &_x[i], 1);
        
#endif
    }
}

// TODO: make this multi-threaded (each synchronizer runs in its own thread)
void multichannelrx::RunChannelizer()
{
    // execute filterbank channelizer as analyzer
    firpfbch_crcf_analyzer_execute(channelizer, x, X);

    // push resulting samples through frame synchronizers one
    // sample at a time
    unsigned int i;
    for (i=0; i<num_channels; i++)
        ofdmflexframesync_execute(framesync[i], &X[i], 1);
}

