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
#include <pthread.h>
#include <liquid/liquid.h>
#include <uhd/usrp/multi_usrp.hpp>

// receiver worker thread
void * ofdmtxrx_rx_worker(void * _arg);

class ofdmtxrx {
public:
    // default constructor
    //  _M              :   OFDM: number of subcarriers
    //  _cp_len         :   OFDM: cyclic prefix length
    //  _taper_len      :   OFDM: taper prefix length
    //  _p              :   OFDM: subcarrier allocation
    //  _callback       :   frame synchronizer callback function
    //  _userdata       :   user-defined data structure
    ofdmtxrx(unsigned int       _M,
             unsigned int       _cp_len,
             unsigned int       _taper_len,
             unsigned char *    _p,
             framesync_callback _callback,
             void *             _userdata);

    // destructor
    ~ofdmtxrx();

    // 
    // transmitter methods
    //
    void set_tx_freq(float _tx_freq);
    void set_tx_rate(float _tx_rate);
    void set_tx_gain_soft(float _tx_gain_soft);
    void set_tx_gain_uhd(float _tx_gain_uhd);
    void set_tx_antenna(char * _tx_antenna);
    void reset_tx();

    // update payload data on a particular channel
    void transmit_packet(unsigned char * _header,
                         unsigned char * _payload,
                         unsigned int    _payload_len,
                         int             _mod,
                         int             _fec0,
                         int             _fec1);
                         // frame generator properties...

    // 
    // receiver methods
    //
    void set_rx_freq(float _rx_freq);
    void set_rx_rate(float _rx_rate);
    void set_rx_gain_uhd(float _rx_gain_uhd);
    void set_rx_antenna(char * _rx_antenna);
    void reset_rx();
    void start_rx();
    void stop_rx();

    //
    // additional methods
    // 
    void debug_enable();
    void debug_disable();

    // specify rx worker method as friend function so that it may
    // gain acess to private members of the class
    friend void * ofdmtxrx_rx_worker(void * _arg);
            
private:
    // set timespec for timeout
    //  _ts         :   pointer to timespec structure
    //  _timeout    :   time before timeout
    void set_timespec(struct timespec * _ts,
                      float             _timeout);

    // OFDM properties
    unsigned int M;                 // number of subcarriers
    unsigned int cp_len;            // cyclic prefix length
    unsigned int taper_len;         // taper length
    ofdmflexframegenprops_s fgprops;// frame generator properties

    // transmitter objects
    ofdmflexframegen fg;            // frame generator object
    std::complex<float> * fgbuffer; // frame generator output buffer [size: M + cp_len x 1]
    unsigned int fgbuffer_len;      // length of frame generator buffer
    float tx_gain;                  // soft transmit gain (linear)
#if 0
    pthread_t tx_process;           // transmit thread
    pthread_mutex_t tx_mutex;       // transmit mutex
#endif

    // receiver objects
    ofdmflexframesync fs;           // frame synchronizer object
    pthread_t rx_process;           // receive thread
    pthread_mutex_t rx_mutex;       // receive mutex
    pthread_cond_t  rx_cond;        // receive condition
    bool rx_running;                // is receiver running? (physical receiver)
    bool rx_thread_running;         // is receiver thread running?
    bool debug_enabled;             // is debugging enabled?

    // RF objects and properties
    uhd::usrp::multi_usrp::sptr usrp_tx;
    uhd::usrp::multi_usrp::sptr usrp_rx;
    uhd::tx_metadata_t          metadata_tx;
};

#if 0
// 
int ofdmtxrx_callback(unsigned char *  _header,
                      int              _header_valid,
                      unsigned char *  _payload,
                      unsigned int     _payload_len,
                      int              _payload_valid,
                      framesyncstats_s _stats,
                      void *           _userdata);
#endif

#endif // __OFDMTXRX_H__

