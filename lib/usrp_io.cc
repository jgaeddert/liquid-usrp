/*
 * Copyright (c) 2007, 2008, 2009, 2010 Joseph Gaeddert
 * Copyright (c) 2007, 2008, 2009, 2010 Virginia Polytechnic
 *                                      Institute & State University
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
#include <stdio.h>
#include <complex>
#include <usrp_standard.h>
#include <usrp_prims.h>
#include <usrp_dbid.h>
#include <usrp_bytesex.h>

#include "usrp_io.h"
#include "usrp_rx_gain_correction.h"

#define USRP_IO_RX_GAIN     (1.0f / 64.0f)
#define USRP_IO_TX_GAIN     (8000.0f)

#if USRP_VERSION < 3
#else
#  include <byteswap.h>
#endif

// default constructor
usrp_io::usrp_io()
{
    // flags
    use_complex = true;
    rx_active = false;  // rx thread controller flag
    tx_active = false;  // tx thread controller flag

    rx_running = false; // rx thread status flag
    tx_running = false; // tx thread status flag

    initialize();

    // buffering
    tx_buffer_length = 512;
    rx_buffer_length = 512;

    tx_port_buffer = new std::complex<float>[tx_buffer_length*sizeof(short)];
    rx_port_buffer = new std::complex<float>[rx_buffer_length*sizeof(short)];

    tx_buffer = new short[2*tx_buffer_length*sizeof(short)];
    rx_buffer = new short[2*rx_buffer_length*sizeof(short)];

    // ports
    port_tx = gport_create(4*tx_buffer_length, sizeof(std::complex<float>));
    port_rx = gport_create(4*tx_buffer_length, sizeof(std::complex<float>));

    port_resamp_tx = gport_create(4*tx_buffer_length, sizeof(std::complex<float>));
    port_resamp_rx = gport_create(4*tx_buffer_length, sizeof(std::complex<float>));

    // resampling
    tx_resamp_rate = 1.0f;
    rx_resamp_rate = 1.0f;
    tx_resamp = resamp_crcf_create(tx_resamp_rate,7,0.8f,60.0f,32);
    rx_resamp = resamp_crcf_create(rx_resamp_rate,7,0.8f,60.0f,32);

#if USRPIO_USE_DC_BLOCKER
    m_hat = 0.0f;
    beta = 1e-3f;
    alpha = 1.0f - beta;
#endif
}

usrp_io::~usrp_io()
{
    // force stop tx/rx
    stop_tx(0);
    stop_rx(0);

    // signal eom on all ports
    gport_signal_eom(port_tx);
    gport_signal_eom(port_rx);

    gport_signal_eom(port_resamp_tx);
    gport_signal_eom(port_resamp_rx);

    
    // wait for tx/rx inactive
    printf("waiting for tx/rx to stop running...\n");
    unsigned int n=5000; // timeout
    while ( (tx_running || rx_running) && n > 0) {
        usleep(1000);
        n--;
    }
    if (n==0)
        std::cerr << "warning: usrp_io::~usrp_io(), tx/rx still running during destructor invocation!" << std::endl;

    // destroy ports
    gport_destroy(port_tx);
    gport_destroy(port_rx);
    gport_destroy(port_resamp_tx);
    gport_destroy(port_resamp_rx);

    // destroy resampling objects
    resamp_crcf_destroy(tx_resamp);
    resamp_crcf_destroy(rx_resamp);

    // destroy buffers
    delete [] tx_port_buffer;
    delete [] rx_port_buffer;

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
    pthread_create(&tx_resamp_thread, NULL, usrp_io_tx_resamp_process, this);
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
    pthread_create(&rx_resamp_thread, NULL, usrp_io_rx_resamp_process, this);
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

#if USRP_VERSION < 3
    // set the daughterboard frequency
    float db_lo_offset  = -8e6;
    float db_lo_freq    = 0.0f;
    float db_lo_freq_set = _freq + db_lo_offset;
    tx_db0->set_db_freq(db_lo_freq_set, db_lo_freq);
    float ddc_freq = _freq - db_lo_freq;
    usrp_tx->set_tx_freq(_channel, ddc_freq);
#else
    db_base_sptr db;
    if (_channel == 0) {
        db = tx_db0;
    } else if (_channel == 1) {
        db = tx_db1;
    } else {
        fprintf(stderr,"error: usrp_io::set_tx_Freq(), invalid channel: %d\n", _channel);
        exit(1);
    }

    usrp_tune_result result;
    if (!usrp_tx->tune(_channel, db, _freq, &result)) {
        fprintf(stderr,"error: usrp_io::set_tx_freq(), tune failed\n");
        exit(1);
    }

    // print debug info
    printf("usrp tx tune result:\n");
    printf("    baseband freq   :   %12.4e\n", result.baseband_freq);
    printf("    dxc_freq        :   %12.4e\n", result.dxc_freq);
    printf("    residual_freq   :   %12.4e\n", result.residual_freq);
    printf("    inverted        :   %u\n", result.inverted);
#endif

    // TODO: store local oscillator and ddc frequencies internally
}
void usrp_io::set_rx_freq(int _channel, float _freq)
{
#if USRP_VERSION < 3
    // TODO: check daughterboard capabilities

    // set the daughterboard frequency
    float db_lo_offset  = -8e6;
    float db_lo_freq    = 0.0f;
    float db_lo_freq_set = _freq + db_lo_offset;
    rx_db0->set_db_freq(db_lo_freq_set, db_lo_freq);
    float ddc_freq = _freq - db_lo_freq;
    usrp_rx->set_rx_freq(_channel, ddc_freq);

    // TODO: store local oscillator and ddc frequencies internally

#else
    db_base_sptr db;
    if (_channel == 0) {
        db = rx_db0;
    } else if (_channel == 1) {
        db = rx_db1;
    } else {
        fprintf(stderr,"error: usrp_io::set_rx_Freq(), invalid channel: %d\n", _channel);
        exit(1);
    }

    usrp_tune_result result;
    if (!usrp_rx->tune(_channel, db, _freq, &result)) {
        fprintf(stderr,"error: usrp_io::set_rx_freq(), tune failed\n");
        exit(1);
    }

    // print debug info
    printf("usrp rx tune result:\n");
    printf("    baseband freq   :   %12.4e\n", result.baseband_freq);
    printf("    dxc_freq        :   %12.4e\n", result.dxc_freq);
    printf("    residual_freq   :   %12.4e\n", result.residual_freq);
    printf("    inverted        :   %u\n", result.inverted);
#endif

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

float usrp_io::get_tx_samplerate()
{
    float interp_rate = (float) (usrp_tx->interp_rate());
    return 32e6f / interp_rate * tx_resamp_rate;
}

float usrp_io::get_rx_samplerate()
{
    float decim_rate = (float) (usrp_rx->decim_rate());
    return 16e6f / decim_rate * rx_resamp_rate;
}

void usrp_io::set_tx_samplerate(float _tx_samplerate)
{
    // compute interpolation rate
    unsigned int interp_rate = (unsigned int)(64e6 / _tx_samplerate);

    // ensure multiple of 4
    interp_rate = (interp_rate >> 2) << 2;

    // check special condition : interp_rate = 44
    if (interp_rate == 44) interp_rate = 48;

    // compute usrp sampling rate
    float usrp_tx_samplerate = 64e6f / (float)interp_rate;

    // compute arbitrary resampling rate
    tx_resamp_rate = usrp_tx_samplerate / _tx_samplerate;
    resamp_crcf_setrate(tx_resamp, tx_resamp_rate);
    usleep(200000); // NOTE: this sleep is necessary before re-setting the interp
                    //       rate so that bad things don't happen on the RF output
    usrp_tx->set_interp_rate(interp_rate);

    printf("usrp_io::set_tx_samplerate() %8.4f kHz = %8.4f kHz * %8.6f (interp %u)\n",
            _tx_samplerate * 1e-3f,
            usrp_tx_samplerate * 1e-3f,
            1.0f / tx_resamp_rate,
            interp_rate);
}

void usrp_io::set_rx_samplerate(float _rx_samplerate)
{
    // compute decimation rate
    unsigned int decim_rate = (unsigned int)(32e6 / _rx_samplerate);

    // ensure multiple of 2
    decim_rate = (decim_rate >> 1) << 1;

    // compute usrp sampling rate
    float usrp_rx_samplerate = 32e6f / (float)decim_rate;

    // compute arbitrary resampling rate
    rx_resamp_rate = _rx_samplerate / usrp_rx_samplerate;
    resamp_crcf_setrate(rx_resamp, rx_resamp_rate);
    usrp_rx->set_decim_rate(decim_rate);

    printf("usrp_io::set_rx_samplerate() %8.4f kHz = %8.4f kHz * %8.6f (decim %u)\n",
            _rx_samplerate * 1e-3f,
            usrp_rx_samplerate * 1e-3f,
            rx_resamp_rate,
            decim_rate);
}

// other properties
void usrp_io::enable_auto_tx(int _channel)
{
    tx_db0->set_auto_tr(true);
    rx_db0->set_auto_tr(true);
}

void usrp_io::disable_auto_tx(int _channel)
{
    tx_db0->set_auto_tr(false);
    rx_db0->set_auto_tr(false);
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

    // check for rx daughterboards and assign first subdevice
    int rx_db0_id = usrp_rx->daughterboard_id(0);
    int rx_db1_id = usrp_rx->daughterboard_id(1);

#if USRP_VERSION < 3
    if (rx_db0_id == USRP_DBID_FLEX_400_RX_MIMO_B) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400_RX_MIMO_B\n");
        rx_db0 = new db_flex400_rx_mimo_b(usrp_rx,0);
    } else {
        std::cerr << "error: usrp_io::initialize(), use usrp db flex 400 rx MIMO B" << std::endl;
        throw 0;
    }
#else
    rx_db0 = usrp_rx->db(0)[0];
    rx_db1 = usrp_rx->db(1)[0];
#endif

    std::cout << "usrp daughterboard rx slot 0 : " << usrp_dbid_to_string(rx_db0_id) << std::endl;
    std::cout << "usrp daughterboard rx slot 1 : " << usrp_dbid_to_string(rx_db1_id) << std::endl;


    // check for tx daughterboards
    int tx_db0_id = usrp_tx->daughterboard_id(0);
    int tx_db1_id = usrp_tx->daughterboard_id(1);

#if USRP_VERSION < 3
    if (tx_db0_id == USRP_DBID_FLEX_400_TX_MIMO_B) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400_TX_MIMO_B\n");
        tx_db0 = new db_flex400_tx_mimo_b(usrp_tx,0);
    } else {
        std::cerr << "error: usrp_io::initialize(), use usrp db flex 400 tx MIMO B" << std::endl;
        throw 0;
    }
#else
    rx_db0 = usrp_rx->db(0)[0];
    rx_db1 = usrp_rx->db(1)[0];
#endif

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
    int gport_eom=0;

    // set internal tx running flag
    usrp->tx_running = true;

    // start data transfer
    usrp->usrp_tx->start();

    while (usrp->tx_active && !gport_eom) {
        // wait for data
        gport_eom =
        gport_consume(usrp->port_tx,
                      (void*)(usrp->tx_port_buffer),
                      usrp->tx_buffer_length);

        if (gport_eom) break;

        // convert to short
        for (unsigned int i=0; i<usrp->tx_buffer_length; i++) {
            usrp->tx_buffer[2*i+0] = (short)(usrp->tx_port_buffer[i].real() * usrp->tx_gain);
            usrp->tx_buffer[2*i+1] = (short)(usrp->tx_port_buffer[i].imag() * usrp->tx_gain);
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
    }

    // stop data transfer
    usrp->usrp_tx->stop();

    std::cout << "usrp_io_tx_process() terminating" << std::endl;

    // clear internal tx running flag
    usrp->tx_running = false;

    pthread_exit(NULL);
}

void* usrp_io_rx_process(void * _u)
{
    std::cout << "usrp_io_rx_process() invoked" << std::endl;
    usrp_io * usrp = (usrp_io*) _u;

    // local variables
    int rc;
    bool overrun;
    int gport_eom=0;

    // set internal rx running flag
    usrp->rx_running = true;

    // start data transfer
    usrp->usrp_rx->start();

    while (usrp->rx_active && !gport_eom) {
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

        // convert to complex float
        for (unsigned int i=0; i<usrp->rx_buffer_length; i++) {
            usrp->rx_port_buffer[i].real() =  (float)(usrp->rx_buffer[2*i+0]) * usrp->rx_gain;
            usrp->rx_port_buffer[i].imag() = -(float)(usrp->rx_buffer[2*i+1]) * usrp->rx_gain;
        }

#if USRPIO_USE_DC_BLOCKER
        // dc blocker
        for (unsigned int i=0; i<usrp->rx_buffer_length; i++) {
            usrp->m_hat = (usrp->alpha)*(usrp->m_hat) + (usrp->beta) * usrp->rx_port_buffer[i];
            usrp->rx_port_buffer[i] -= usrp->m_hat;
        }
#endif
        gport_eom =
        gport_produce(usrp->port_rx,
                      (void*)(usrp->rx_port_buffer),
                      usrp->rx_buffer_length);
    }

    // stop data transfer
    usrp->usrp_rx->stop();

    std::cout << "usrp_io_rx_process() terminating" << std::endl;

    // clear internal rx running flag
    usrp->rx_running = false;

    pthread_exit(NULL);
}


void* usrp_io_tx_resamp_process(void * _u)
{
    std::cout << "usrp_io_tx_resamp_process() invoked" << std::endl;
    usrp_io * usrp = (usrp_io*) _u;

    // local buffers
    unsigned int n = usrp->tx_buffer_length;
    std::complex<float> data_in[n];
    std::complex<float> data_out[2*n];
    unsigned int num_written;
    unsigned int num_written_total;
    unsigned int i;
    int gport_eom=0;

    while (usrp->tx_active && !gport_eom) {
        // get data from port_resamp_tx
        gport_eom =
        gport_consume(usrp->port_resamp_tx,
                      (void*)data_in,
                      n);

        if (gport_eom) break;

        // run resampler
        num_written_total=0;
        for (i=0; i<n; i++) {
            resamp_crcf_execute(usrp->tx_resamp,
                                data_in[i],
                                &data_out[num_written_total],
                                &num_written);
            num_written_total += num_written;
        }

        // push data to usrp thread
        gport_eom =
        gport_produce(usrp->port_tx,
                      (void*)data_out,
                      num_written_total);
    }

    std::cout << "usrp_io_tx_resamp_process() terminating" << std::endl;
    pthread_exit(NULL);
}

void* usrp_io_rx_resamp_process(void * _u)
{
    std::cout << "usrp_io_rx_resamp_process() invoked" << std::endl;
    usrp_io * usrp = (usrp_io*) _u;

    // local buffers
    unsigned int n = usrp->rx_buffer_length;
    std::complex<float> data_in[n];
    std::complex<float> data_out[2*n];
    unsigned int num_written;
    unsigned int num_written_total;
    unsigned int i;
    int gport_eom=0;

    while (usrp->rx_active && !gport_eom) {
        // get data from port_rx
        gport_eom =
        gport_consume(usrp->port_rx,
                      (void*)data_in,
                      n);

        if (gport_eom) break;

        // run resampler
        num_written_total=0;
        for (i=0; i<n; i++) {
            resamp_crcf_execute(usrp->rx_resamp,
                                data_in[i],
                                &data_out[num_written_total],
                                &num_written);
            num_written_total += num_written;
        }

        // push data to output
        gport_eom =
        gport_produce(usrp->port_resamp_rx,
                      (void*)data_out,
                      num_written_total);
    }

    std::cout << "usrp_io_rx_resamp_process() terminating" << std::endl;
    pthread_exit(NULL);
}

