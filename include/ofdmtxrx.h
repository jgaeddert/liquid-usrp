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
// ofdmtxrx.h
//
// OFDM transceiver class for simple interface with USRP
//

#ifndef __OFDMTXRX_H__
#define __OFDMTXRX_H__

#include <complex>
#include <liquid/liquid.h>
#include <uhd/usrp/multi_usrp.hpp>

class ofdmtxrx {
public:
    // default constructor
    //  _M              :   OFDM: number of subcarriers
    //  _cp_len         :   OFDM: cyclic prefix length
    //  _taper_len      :   OFDM: taper prefix length
    ofdmtxrx(unsigned int _M,
             unsigned int _cp_len,
             unsigned int _taper_len);

    // destructor
    ~ofdmtxrx();

    // reset transceiver
    void Reset();

    // set properties
    void SetFrequency(double _frequency);
    void SetSampleRate(double _sample_rate);
    void SetHardwareGainTx(double _uhd_gain_tx);
    void SetHardwareGainRx(double _uhd_gain_rx);
    void SetSoftwareGainTx(double _gain_tx);
    void SetSoftwareGainRx(double _gain_rx);

    // start/stop
    void StartTransmitter();
    void StopTransmitter();
    void StartReceiver();
    void StopReceiver();

    // update payload data on a particular channel
    void TransmitPacket(unsigned char * _header,
                        unsigned char * _payload,
                        unsigned int    _payload_len,
                        int             _mod,
                        int             _fec0,
                        int             _fec1);
                        // frame generator properties...

    // receive packet with timeout
    //  _timeout        :   timeout (seconds)
    //  _header
    //  ...
    bool ReceivePacket(double             _timeout,
                       unsigned char **   _header,
                       int  *             _header_valid,
                       unsigned char **   _payload,
                       unsigned int  *    _payload_len,
                       int  *             _payload_valid,
                       framesyncstats_s * _stats);
            
private:
    // generate frame samples from internal frame generator
    //void GenerateFrameSamples();

    // OFDM properties
    unsigned int M;                 // number of subcarriers
    unsigned int cp_len;            // cyclic prefix length
    unsigned int taper_len;         // taper length
    ofdmflexframegenprops_s fgprops;// frame generator properties

    // transmitter objects
    ofdmflexframegen framegen;      // frame generator object
    std::complex<float> * fgbuffer; // frame generator output buffer [size: M + cp_len x 1]
    unsigned int fgbuffer_len;      // length of frame generator buffer

    // receiver objects
    ofdmflexframesync framesync;    // frame generator object
#if 0
    std::complex<float> * fsbuffer; // frame generator output buffer [size: M + cp_len x 1]
    unsigned int fgbuffer_len;      // length of frame generator buffer
#endif

    // RF objects and properties
    double frequency;               // carrier frequency
    double sample_rate;             // host sample rate
    double usrp_sample_rate;        // usrp sample rate
    uhd::usrp::multi_usrp::sptr usrp_tx;
    uhd::usrp::multi_usrp::sptr usrp_rx;
};

// 
int ofdmtxrx_callback(unsigned char *  _header,
                      int              _header_valid,
                      unsigned char *  _payload,
                      unsigned int     _payload_len,
                      int              _payload_valid,
                      framesyncstats_s _stats,
                      void *           _userdata);

#endif // __OFDMTXRX_H__

