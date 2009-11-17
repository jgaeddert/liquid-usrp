/*
 * Copyright (c) 2007, 2009 Joseph Gaeddert
 * Copyright (c) 2007, 2009 Virginia Polytechnic Institute & State University
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

#include <iostream>
#include <usrp_standard.h>
#include <usrp_prims.h>
#include <usrp_dbid.h>

#include "usrp_io.h"
#include "flex.h"
#include "basic.h"
#include "lf.h"
#include "dbsrx.h"
#include "tvrx.h"
#include "usrp_rx_gain_correction.h"

#define USRP_IO_RX_GAIN     (1.0f / 64.0f)
#define USRP_IO_TX_GAIN     (8000.0f)

// default constructor
usrp_io::usrp_io()
{
    // flags
    use_complex = true;
    rx_active = false;
    tx_active = false;

    initialize();

    // buffering
    tx_buffer_length = 512;
    rx_buffer_length = 512;

    tx_buffer = new short[2*tx_buffer_length*sizeof(short)];
    rx_buffer = new short[2*tx_buffer_length*sizeof(short)];

    port_tx = gport_create(2048, sizeof(std::complex<float>));
    port_rx = gport_create(2048, sizeof(std::complex<float>));
}

usrp_io::~usrp_io()
{
    // destroy ports
    gport_destroy(port_tx);
    gport_destroy(port_rx);

    // delete usrp_rx and usrp_tx objects
    // TODO : figure out why this segfaults
    //delete usrp_rx;
    //delete usrp_tx;

    delete [] tx_buffer;
    delete [] rx_buffer;
}

// start/stop
void usrp_io::start_tx(int _channel)
{
    if (_channel != 0) {
        std::cerr << "error: usrp_io::start_tx(), only channel 0 currently supported" << std::endl;
        throw 0;
    } else if (tx_active) {
        std::cerr << "error: usrp_io::start_tx(), tx active" << std::endl;
        throw 0;
    }
    tx_active = true;
    pthread_create(&tx_thread, NULL, usrp_io_tx_process, this);
}

void usrp_io::start_rx(int _channel)
{
    if (_channel != 0) {
        std::cerr << "error: usrp_io::start_rx(), only channel 0 currently supported" << std::endl;
        throw 0;
    } else if (rx_active) {
        std::cerr << "error: usrp_io::start_rx(), rx active" << std::endl;
        throw 0;
    }
    rx_active = true;
    pthread_create(&rx_thread, NULL, usrp_io_rx_process, this);
}

// gain
void usrp_io::get_tx_gain(int _channel, float &_gain) {}
void usrp_io::get_rx_gain(int _channel, float &_gain) {}
void usrp_io::set_tx_gain(int _channel, float _gain) {}
void usrp_io::set_rx_gain(int _channel, float _gain) {}

// frequency
void usrp_io::get_tx_freq(int _channel, float &_freq) { _freq = 0.0f; }
void usrp_io::get_rx_freq(int _channel, float &_freq) { _freq = 0.0f; }
void usrp_io::set_tx_freq(int _channel, float _freq)
{
    // TODO: check daughterboard capabilities

    // set the daughterboard frequency
    float db_lo_offset  = -8e6;
    float db_lo_freq    = 0.0f;
    float db_lo_freq_set = _freq + db_lo_offset;
    tx_db0->set_db_freq(db_lo_freq_set, db_lo_freq);
    float ddc_freq = _freq - db_lo_freq;
    usrp_tx->set_tx_freq(_channel, ddc_freq);

    // TODO: store local oscillator and ddc frequencies internally
}
void usrp_io::set_rx_freq(int _channel, float _freq)
{
    // TODO: check daughterboard capabilities

    // set the daughterboard frequency
    float db_lo_offset  = -8e6;
    float db_lo_freq    = 0.0f;
    float db_lo_freq_set = _freq + db_lo_offset;
    rx_db0->set_db_freq(db_lo_freq_set, db_lo_freq);
    float ddc_freq = _freq - db_lo_freq;
    usrp_rx->set_rx_freq(_channel, ddc_freq);

    // TODO: store local oscillator and ddc frequencies internally
}

// decimation/interpolation
unsigned int usrp_io::get_tx_interp()
{
    return usrp_tx->interp_rate();
}

unsigned int usrp_io::get_rx_decim()
{
    return usrp_rx->decim_rate();
}

void usrp_io::set_tx_interp(int _interp)
{
    usrp_tx->set_interp_rate(_interp);
}

void usrp_io::set_rx_decim(int _decim)
{
    usrp_rx->set_decim_rate(_decim);

    // adjust gain
    rx_gain = USRP_IO_RX_GAIN * usrp_rx_gain_correction(_decim);
}

// initialization methods
void usrp_io::initialize()
{
    std::cout << "initializing usrp..." << std::endl;

    // create rx object
    usrp_rx = usrp_standard_rx::make(0, 256);
    if (!usrp_rx) {
        std::cerr << "error: usrp_io::initialize(), could not create usrp rx"
            << std::endl;
        throw 0;
    }

    // create tx object
    usrp_tx = usrp_standard_tx::make(0, 512);
    if (!usrp_tx) {
        std::cerr << "error: usrp_io::initialize(), could not create usrp tx"
            << std::endl;
        throw 0;
    }

    // check for rx daughterboards
    int rx_db0_id = usrp_rx->daughterboard_id(0);
    int rx_db1_id = usrp_rx->daughterboard_id(1);

    if (rx_db0_id == USRP_DBID_FLEX_400_RX_MIMO_B) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400_RX_MIMO_B\n");
        rx_db0 = new db_flex400_rx_mimo_b(usrp_rx,0);
    } else {
        std::cerr << "error: usrp_io::initialize(), use usrp db flex 400 rx MIMO B" << std::endl;
        throw 0;
    }

    std::cout << "usrp daughterboard rx slot 0 : " << usrp_dbid_to_string(rx_db0_id) << std::endl;
    std::cout << "usrp daughterboard rx slot 1 : " << usrp_dbid_to_string(rx_db1_id) << std::endl;


    // check for tx daughterboards
    int tx_db0_id = usrp_tx->daughterboard_id(0);
    int tx_db1_id = usrp_tx->daughterboard_id(1);

    if (tx_db0_id == USRP_DBID_FLEX_400_TX_MIMO_B) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400_TX_MIMO_B\n");
        tx_db0 = new db_flex400_tx_mimo_b(usrp_tx,0);
    } else {
        std::cerr << "error: usrp_io::initialize(), use usrp db flex 400 tx MIMO B" << std::endl;
        throw 0;
    }

    std::cout << "usrp daughterboard tx slot 0 : " << usrp_dbid_to_string(tx_db0_id) << std::endl;
    std::cout << "usrp daughterboard tx slot 1 : " << usrp_dbid_to_string(tx_db1_id) << std::endl;

    // default: set tx_enable
    tx_db0->set_enable(true);

    // defaults
    usrp_rx->set_nchannels(1);
    usrp_tx->set_nchannels(1);

    // gains
    rx_gain = USRP_IO_RX_GAIN * usrp_rx_gain_correction(256);
    tx_gain = USRP_IO_TX_GAIN;
}

// threading functions
void* usrp_io_tx_process(void * _u)
{
    std::cout << "usrp_io_tx_process() invoked" << std::endl;
    usrp_io * usrp = (usrp_io*) _u;

    // local variables
    int rc;
    bool underrun;

    // start data transfer
    usrp->usrp_tx->start();

    std::complex<float> * data;

    while (usrp->tx_active) {
        // wait for data
        data = (std::complex<float>*) gport_consumer_lock(usrp->port_tx,usrp->tx_buffer_length);

        // convert to short
        for (unsigned int i=0; i<usrp->tx_buffer_length; i++) {
            usrp->tx_buffer[2*i+0] = (short)(data[i].real() * usrp->tx_gain);
            usrp->tx_buffer[2*i+1] = (short)(data[i].imag() * usrp->tx_gain);
        }

        // write data
        rc = usrp->usrp_tx->write(usrp->tx_buffer, 2*usrp->tx_buffer_length*sizeof(short), &underrun);

        if (rc < 0) {
            std::cerr << "error: usrp_io_tx_process(), tx error" << std::endl;
            throw 0;
        } else if (rc != (int)(2*usrp->tx_buffer_length*sizeof(short)) ) {
            std::cerr << "warning: usrp_io_tx_process(), usrp attempted to write "
                      << usrp->tx_buffer_length << " values ("
                      << rc << " actually written)" << std::endl;
        }

        if (underrun)
            std::cerr << "underrun" << std::endl;

        // unlock data
        gport_consumer_unlock(usrp->port_tx, usrp->tx_buffer_length);
    }

    // stop data transfer
    usrp->usrp_tx->stop();

    std::cout << "usrp_io_tx_process() terminating" << std::endl;
    pthread_exit(NULL);
}

void* usrp_io_rx_process(void * _u)
{
    std::cout << "usrp_io_rx_process() invoked" << std::endl;
    usrp_io * usrp = (usrp_io*) _u;

    // local variables
    int rc;
    bool overrun;
    std::complex<float> * data;

    // start data transfer
    usrp->usrp_rx->start();

    while (usrp->rx_active) {
        // read data
        rc = usrp->usrp_rx->read(usrp->rx_buffer, 2*usrp->rx_buffer_length*sizeof(short), &overrun);

        if (rc < 0) {
            std::cerr << "error: usrp_io_rx_process(), rx error ("
                      << rc << ")" << std::endl;
            throw 0;
        } else if (rc != (int)(2*usrp->rx_buffer_length*sizeof(short)) ) {
            std::cerr << "warning: usrp_io_rx_process(), usrp attempted to write "
                      << usrp->rx_buffer_length << " values ("
                      << rc << " actually written)" << std::endl;
        }

        if (overrun)
            std::cerr << "overrun" << std::endl;

        data = (std::complex<float>*) gport_producer_lock(usrp->port_rx,usrp->rx_buffer_length);

        // convert to complex float
        for (unsigned int i=0; i<usrp->rx_buffer_length; i++) {
            data[i].real() =  (float)(usrp->rx_buffer[2*i+0]) * usrp->rx_gain;
            data[i].imag() = -(float)(usrp->rx_buffer[2*i+1]) * usrp->rx_gain;
        }
        gport_producer_unlock(usrp->port_rx,usrp->rx_buffer_length);

    }

    // stop data transfer
    usrp->usrp_rx->stop();

    std::cout << "usrp_io_rx_process() terminating" << std::endl;
    pthread_exit(NULL);
}


