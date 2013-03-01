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
// multichanneltxrx.cc
//

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <complex>
#include <liquid/liquid.h>

#include "multichanneltxrx.h"

#define DEBUG 0

// debug print
#if DEBUG == 1
#   define dprintf(s) printf(s)
#else
#   define dprintf(s) /* s */
#endif

// default constructor
//  _num_channels   :   number of OFDM channels
//  _M              :   OFDM: number of subcarriers
//  _cp_len         :   OFDM: cyclic prefix length
//  _taper_len      :   OFDM: taper prefix length
//  _p              :   OFDM: subcarrier allocation
//  _callback       :   frame synchronizer callback function
//  _userdata       :   user-defined data structure
multichanneltxrx::multichanneltxrx(unsigned int         _num_channels,
                                   unsigned int         _M,
                                   unsigned int         _cp_len,
                                   unsigned int         _taper_len,
                                   unsigned char *      _p,
                                   framesync_callback * _callback,
                                   void **              _userdata) :
    num_channels(_num_channels),
    mctx(_num_channels, _M, _cp_len, _taper_len, _p),
    mcrx(_num_channels, _M, _cp_len, _taper_len, _p, _userdata, _callback)
{
    // validate input
    if (_num_channels == 0) {
        fprintf(stderr,"error: multichanneltxrx::multichanneltxrx(), number of channels cannot be zero\n");
        throw 0;
    } else if (_M < 8) {
        fprintf(stderr,"error: multichanneltxrx::multichanneltxrx(), number of subcarriers must be at least 8\n");
        throw 0;
    } else if (_cp_len < 1) {
        fprintf(stderr,"error: multichanneltxrx::multichanneltxrx(), cyclic prefix length must be at least 1\n");
        throw 0;
    } else if (_taper_len > _cp_len) {
        fprintf(stderr,"error: multichanneltxrx::multichanneltxrx(), taper length cannot exceed cyclic prefix length\n");
        throw 0;
    }

    // set internal properties
    debug_enabled= false;

    // allocate buffers
    tx_buffer_len = 2*num_channels;
    tx_buffer = (std::complex<float>*) malloc(tx_buffer_len * sizeof(std::complex<float>));
    
    // TODO: create rx buffer

    // create usrp objects
    uhd::device_addr_t dev_addr;
    usrp_tx = uhd::usrp::multi_usrp::make(dev_addr);
    usrp_rx = uhd::usrp::multi_usrp::make(dev_addr);

    // initialize default tx values
    set_tx_freq(462.0e6f);
    set_tx_rate(500e3);
    set_tx_gain_soft(-12.0f);
    set_tx_gain_uhd(40.0f);

    // initialize default rx values
    set_rx_freq(462.0e6f);
    set_rx_rate(500e3);
    set_rx_gain_uhd(20.0f);

    // reset transceiver
    reset_tx();
    reset_rx();

    // create and start rx thread
    rx_running = false;                     // receiver is not running initially
    rx_thread_running = true;               // receiver thread IS running initially
    pthread_mutex_init(&rx_mutex, NULL);    // receiver mutex
    pthread_cond_init(&rx_cond,   NULL);    // receiver condition
    pthread_create(&rx_process,   NULL, multichanneltxrx_rx_worker, (void*)this);
    
    // create and start tx thread
    tx_running = false;                     // receiver is not running initially
    tx_thread_running = true;               // receiver thread IS running initially
    pthread_mutex_init(&tx_mutex, NULL);    // receiver mutex
    pthread_cond_init(&tx_cond,   NULL);    // receiver condition
    pthread_create(&tx_process,   NULL, multichanneltxrx_tx_worker, (void*)this);
}

// destructor
multichanneltxrx::~multichanneltxrx()
{
    dprintf("waiting for process to finish...\n");

    // ensure reciever thread is not running
    if (rx_running) stop_rx();

    // signal condition (tell rx worker to continue)
    dprintf("destructor signaling condition...\n");
    rx_thread_running = false;
    pthread_cond_signal(&rx_cond);

    dprintf("destructor joining rx thread...\n");
    void * exit_status;
    pthread_join(rx_process, &exit_status);

    // destroy threading objects
    dprintf("destructor destroying mutex...\n");
    pthread_mutex_destroy(&rx_mutex);
    dprintf("destructor destroying condition...\n");
    pthread_cond_destroy(&rx_cond);
    
    dprintf("destructor destroying other objects...\n");
    // destroy framing objects

    // free other allocated arrays
    free(tx_buffer);
    
    dprintf("destructor finished\n");
}


// 
// transmitter methods
//

// set transmitter frequency
void multichanneltxrx::set_tx_freq(float _tx_freq)
{
    usrp_tx->set_tx_freq(_tx_freq);
}

// set transmitter sample rate
void multichanneltxrx::set_tx_rate(float _tx_rate)
{
    usrp_tx->set_tx_rate(_tx_rate);
}

// set transmitter software gain
void multichanneltxrx::set_tx_gain_soft(float _tx_gain_soft)
{
    tx_gain = powf(10.0f, _tx_gain_soft/20.0f);
}

// set transmitter hardware (UHD) gain
void multichanneltxrx::set_tx_gain_uhd(float _tx_gain_uhd)
{
    usrp_tx->set_tx_gain(_tx_gain_uhd);
}

// set transmitter antenna
void multichanneltxrx::set_tx_antenna(char * _tx_antenna)
{
    usrp_rx->set_tx_antenna(_tx_antenna);
}

// reset transmitter objects and buffers
void multichanneltxrx::reset_tx()
{
    mctx.Reset();
}

// start transmitter
void multichanneltxrx::start_tx()
{
    dprintf("usrp tx start\n");
    // set tx running flag
    tx_running = true;

    // signal condition (tell tx worker to start)
    pthread_cond_signal(&tx_cond);
}

// stop transmitter
void multichanneltxrx::stop_tx()
{
    dprintf("usrp tx stop\n");

    // set tx running flag
    tx_running = false;
}

// update payload data on a particular channel (non-blocking)
int multichanneltxrx::transmit_packet(unsigned int    _channel,
                                      unsigned char * _header,
                                      unsigned char * _payload,
                                      unsigned int    _payload_len,
                                      int             _mod,
                                      int             _fec0,
                                      int             _fec1)
{
    if (!tx_running) {
        fprintf(stderr,"error: multichanneltxrx:transmit_packet(), transmitter not yet running\n");
        throw 0;
    } else if (_channel >= num_channels) {
        fprintf(stderr,"error: multichanneltxrx:transmit_packet(), invalid channel %u\n", _channel);
        throw 0;
    } else if (!mctx.IsChannelReadyForData(_channel)) {
        fprintf(stderr,"warning: multichanneltxrx:transmit_packet(), channel %u not ready for data\n", _channel);
        return -1;
    }

    // update data on the channel
    mctx.UpdateData(_channel, _header, _payload, _payload_len, _mod, _fec0, _fec1);

    return 0;
}

// is channel available?
bool multichanneltxrx::is_channel_available(unsigned int _channel)
{
    return mctx.IsChannelReadyForData(_channel);
}

// get index of next available channel (blocking)
unsigned int multichanneltxrx::get_available_channel()
{
    // ugly method: simply poll channels until one becomes available
    while (true) {
        unsigned int i;
        for (i=0; i<num_channels; i++) {
            if (mctx.IsChannelReadyForData(i)) {
                // ugly hack to avoid race condition; give internal
                // ofdmflexframegen time to update its internal state
                usleep(20);

                return i;
            }
        }

        // no channels available; sleep for a small time (0.5 ms)
        usleep(500);
    }
}

// wait for a specific channel to become available (blocking)
void multichanneltxrx::wait_for_channel(unsigned int _channel)
{
    // ugly method: simply poll channels until one becomes available
    while (!mctx.IsChannelReadyForData(_channel)) {
        // channel unavailable; sleep for a small time (0.1 ms)
        usleep(100);
    }
}

// wait for all tx channels to be available (blocking, of course)
void multichanneltxrx::wait_for_tx_to_complete()
{
    // ugly method: simply poll channels until all become available
    while (true) {
        unsigned int i;
        bool all_available = true;
        for (i=0; i<num_channels; i++) {
            if (!mctx.IsChannelReadyForData(i))
                all_available = false;
        }

        if (all_available) {
            // all available! return
            return;
        } else {
            // not all channels available; sleep for a small time (0.1 ms)
            usleep(100);
        }
    }
}


// 
// receiver methods
//

// set receiver frequency
void multichanneltxrx::set_rx_freq(float _rx_freq)
{
    usrp_rx->set_rx_freq(_rx_freq);
}

// set receiver sample rate
void multichanneltxrx::set_rx_rate(float _rx_rate)
{
    usrp_rx->set_rx_rate(_rx_rate);
}

// set receiver hardware (UHD) gain
void multichanneltxrx::set_rx_gain_uhd(float _rx_gain_uhd)
{
    usrp_rx->set_rx_gain(_rx_gain_uhd);
}

// set receiver antenna
void multichanneltxrx::set_rx_antenna(char * _rx_antenna)
{
    usrp_rx->set_rx_antenna(_rx_antenna);
}

// reset receiver objects and buffers
void multichanneltxrx::reset_rx()
{
    mcrx.Reset();
}

// start receiver
void multichanneltxrx::start_rx()
{
    dprintf("usrp rx start\n");
    // set rx running flag
    rx_running = true;

    // tell device to start
    usrp_rx->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);

    // signal condition (tell rx worker to start)
    pthread_cond_signal(&rx_cond);
}

// stop receiver
void multichanneltxrx::stop_rx()
{
    dprintf("usrp rx stop\n");
    // set rx running flag
    rx_running = false;

    // tell device to stop
    usrp_rx->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
}

//
// additional methods
//

// enable debugging
void multichanneltxrx::debug_enable()
{
    debug_enabled = true;
}

// disable debugging
void multichanneltxrx::debug_disable()
{
    debug_enabled = false;
}

//
// private methods
//

// set timespec for timeout
//  _ts         :   pointer to timespec structure
//  _timeout    :   time before timeout
void multichanneltxrx::set_timespec(struct timespec * _ts,
                            float             _timeout)
{
    // get current time (timeval)
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    // add offset (timespec)
    _ts->tv_sec  = tv_now.tv_sec;                       // seconds
    _ts->tv_nsec = tv_now.tv_usec*1000 + _timeout*1e9;  // nanoseconds

    // accumulate nanoseconds into seconds
    while (_ts->tv_nsec > 1000000000) {
        _ts->tv_nsec -= 1000000000;
        _ts->tv_sec++;
    }
}

// transmitter worker thread
void * multichanneltxrx_tx_worker(void * _arg)
{
    // type cast input argument as multichanneltxrx object
    multichanneltxrx * txcvr = (multichanneltxrx*) _arg;

    unsigned int i;

    // buffer to hold filterbank channels
    unsigned int tx_buffer_len = 2*txcvr->num_channels;
    std::complex<float> tx_buffer[tx_buffer_len];
    
    // usrp buffer
    std::vector<std::complex<float> > usrp_buffer(256);
    unsigned int usrp_sample_counter = 0;
    
    // transmitter metadata object
    uhd::tx_metadata_t md;
    
    while (txcvr->tx_thread_running) {
        // wait for signal to start; lock mutex
        pthread_mutex_lock(&(txcvr->tx_mutex));

        // this function unlocks the mutex and waits for the condition;
        // once the condition is set, the mutex is again locked
        dprintf("tx_worker waiting for condition...\n");
        //int status =
        pthread_cond_wait(&(txcvr->tx_cond), &(txcvr->tx_mutex));
        dprintf("tx_worker received condition\n");

        // unlock the mutex
        dprintf("tx_worker unlocking mutex\n");
        pthread_mutex_unlock(&(txcvr->tx_mutex));

        // condition given; check state: run or exit
        dprintf("tx_worker running...\n");
        if (!txcvr->tx_running) {
            dprintf("tx_worker finished\n");
            break;
        }

        // set up the metadta flags
        md.start_of_burst = false; // never SOB when continuous
        md.end_of_burst   = false; // 
        md.has_time_spec  = false; // set to false to send immediately

        // reset multichannel transmitter
        txcvr->mctx.Reset();
    
        // run transmitter
        while (txcvr->tx_running) {
            // generate samples
            txcvr->mctx.GenerateSamples(tx_buffer);

            // push resulting samples to USRP
            for (i=0; i<tx_buffer_len; i++) {

                // append to USRP buffer, scaling by software
                usrp_buffer[usrp_sample_counter++] = tx_buffer[i] * txcvr->tx_gain;

                // once USRP buffer is full, reset counter and send to device
                if (usrp_sample_counter==256) {
                    // reset counter
                    usrp_sample_counter=0;

                    // send the result to the USRP
                    txcvr->usrp_tx->get_device()->send(
                        &usrp_buffer.front(), usrp_buffer.size(), md,
                        uhd::io_type_t::COMPLEX_FLOAT32,
                        uhd::device::SEND_MODE_FULL_BUFF
                    );
                }
            }

        } // while tx_running
        
        // send a few extra samples to the device
        // NOTE: this seems necessary to preserve last OFDM symbol in
        //       frame from corruption
        txcvr->usrp_tx->get_device()->send(
            &usrp_buffer.front(), usrp_buffer.size(), md,
            uhd::io_type_t::COMPLEX_FLOAT32,
            uhd::device::SEND_MODE_FULL_BUFF
        );
        
        // send a mini EOB packet
        md.start_of_burst = false;
        md.end_of_burst   = true;

        txcvr->usrp_tx->get_device()->send("", 0, md,
            uhd::io_type_t::COMPLEX_FLOAT32,
            uhd::device::SEND_MODE_FULL_BUFF
        );
        dprintf("tx_worker finished running\n");
    }
    //
    dprintf("tx_worker exiting thread\n");
    pthread_exit(NULL);
}
#if 0
    // vector buffer to send data to device
    std::vector<std::complex<float> > usrp_buffer(tx_buffer_len);

    // set properties
    fgprops.mod_scheme  = _mod;
    fgprops.fec0        = _fec0;
    fgprops.fec1        = _fec1;
    ofdmflexframegen_setprops(fg, &fgprops);

    // assemble frame
    ofdmflexframegen_assemble(fg, _header, _payload, _payload_len);

    // generate a single OFDM frame
    bool last_symbol=false;
    unsigned int i;
    while (!last_symbol) {

        // generate symbol
        last_symbol = ofdmflexframegen_writesymbol(fg, tx_buffer);

        // copy symbol and apply gain
        for (i=0; i<tx_buffer_len; i++)
            usrp_buffer[i] = tx_buffer[i] * tx_gain;

        // send samples to the device
        usrp_tx->get_device()->send(
            &usrp_buffer.front(), usrp_buffer.size(),
            metadata_tx,
            uhd::io_type_t::COMPLEX_FLOAT32,
            uhd::device::SEND_MODE_FULL_BUFF
        );

    } // while loop

#endif


// receiver worker thread
void * multichanneltxrx_rx_worker(void * _arg)
{
    // type cast input argument as multichanneltxrx object
    multichanneltxrx * txcvr = (multichanneltxrx*) _arg;

    // set up receive buffer
    const size_t max_samps_per_packet = txcvr->usrp_rx->get_device()->get_max_recv_samps_per_packet();
    std::vector<std::complex<float> > buffer(max_samps_per_packet);

    // receiver metadata object
    uhd::rx_metadata_t md;

    while (txcvr->rx_thread_running) {
        // wait for signal to start; lock mutex
        pthread_mutex_lock(&(txcvr->rx_mutex));

        // this function unlocks the mutex and waits for the condition;
        // once the condition is set, the mutex is again locked
        dprintf("rx_worker waiting for condition...\n");
        //int status =
        pthread_cond_wait(&(txcvr->rx_cond), &(txcvr->rx_mutex));
        dprintf("rx_worker received condition\n");

        // unlock the mutex
        dprintf("rx_worker unlocking mutex\n");
        pthread_mutex_unlock(&(txcvr->rx_mutex));

        // condition given; check state: run or exit
        dprintf("rx_worker running...\n");
        if (!txcvr->rx_running) {
            dprintf("rx_worker finished\n");
            break;
        }

        // run receiver
        while (txcvr->rx_running) {

            // grab data from device
            //dprintf("rx_worker waiting for samples...\n");
            size_t num_rx_samps = txcvr->usrp_rx->get_device()->recv(
                &buffer.front(), buffer.size(), md,
                uhd::io_type_t::COMPLEX_FLOAT32,
                uhd::device::RECV_MODE_ONE_PACKET
            );
            //dprintf("rx_worker processing samples...\n");

            // ignore error codes for now
#if 0
            // 'handle' the error codes
            switch(md.error_code){
            case uhd::rx_metadata_t::ERROR_CODE_NONE:
            case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
                break;

            default:
                std::cerr << "Error code: " << md.error_code << std::endl;
                std::cerr << "Unexpected error on recv, exit test..." << std::endl;
                //return 1;
                //std::cerr << "rx_worker exiting prematurely" << std::endl;
                //pthread_exit(NULL);
            }
#endif

            // push data through frame synchronizer
            // TODO : use arbitrary resampler?
            // TODO : operate on block of samples
            unsigned int j;
            for (j=0; j<num_rx_samps; j++) {
                // grab sample from usrp buffer
                std::complex<float> usrp_sample = buffer[j];

                // push resulting samples through multi-channel receiver
                txcvr->mcrx.Execute(&usrp_sample, 1);
            }

        } // while rx_running
        dprintf("rx_worker finished running\n");

    } // while true
    
    //
    dprintf("rx_worker exiting thread\n");
    pthread_exit(NULL);
}

