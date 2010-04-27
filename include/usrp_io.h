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

//
//
//

#include <pthread.h>
#include <liquid/liquid.h>

#define USRPIO_USE_DC_BLOCKER 0

// forward declaration of classes
class ossie_db_base;    // ossie_db_base.h
class usrp_standard_rx; // usrp_standard.h
class usrp_standard_tx; // usrp_standard.h

// threading functions
void* usrp_io_tx_process(void * _u);
void* usrp_io_rx_process(void * _u);
void* usrp_io_tx_resamp_process(void * _u);
void* usrp_io_rx_resamp_process(void * _u);

class usrp_io
{
    // friend functions allow private access
    friend void* usrp_io_tx_process(void * _u);
    friend void* usrp_io_rx_process(void * _u);
    friend void* usrp_io_tx_resamp_process(void * _u);
    friend void* usrp_io_rx_resamp_process(void * _u);

public:
    // default constructor
    usrp_io();
    ~usrp_io();

    // start/stop
    void start_tx(int _channel);
    void start_rx(int _channel);
    void stop_tx(int _channel) { tx_active = false; }
    void stop_rx(int _channel) { rx_active = false; }

    // gain
    void get_tx_gain(int _channel, float &_gain);
    void get_rx_gain(int _channel, float &_gain);
    void set_tx_gain(int _channel, float _gain);
    void set_rx_gain(int _channel, float _gain);

    // frequency
    void get_tx_freq(int _channel, float &_freq);
    void get_rx_freq(int _channel, float &_freq);
    void set_tx_freq(int _channel, float _freq);
    void set_rx_freq(int _channel, float _freq);

    // decimation/interpolation [deprecated]
    unsigned int get_tx_interp();
    unsigned int get_rx_decim();
    void set_tx_interp(int _interp);
    void set_rx_decim(int _decim);

    // sample rate
    float get_tx_samplerate();
    float get_rx_samplerate();
    void set_tx_samplerate(float _tx_rate);
    void set_rx_samplerate(float _rx_rate);

    // other properties
    void enable_auto_tx(int _channel);
    void disable_auto_tx(int _channel);

    // port handling
    gport get_tx_port(int _channel) { return port_resamp_tx; }
    gport get_rx_port(int _channel) { return port_resamp_rx; }

protected:
    // initialization methods
    void initialize();

    // gr/usrp objects
    usrp_standard_rx * usrp_rx;
    usrp_standard_tx * usrp_tx;

    // daughterboards
    ossie_db_base * rx_db0;
    ossie_db_base * rx_db1;
    ossie_db_base * tx_db0;
    ossie_db_base * tx_db1;

    // flags
    bool use_complex;
    bool rx_active;     // rx thread controller flag
    bool tx_active;     // tx thread controller flag
    bool rx_running;    // rx thread status flag
    bool tx_running;    // tx thread status flag

    // tx/rx processing threads
    pthread_t tx_thread;
    pthread_t rx_thread;
    pthread_t tx_resamp_thread;
    pthread_t rx_resamp_thread;

    // internal buffering
    unsigned int tx_buffer_length;
    unsigned int rx_buffer_length;
    short * tx_buffer;
    short * rx_buffer;
    std::complex<float> * tx_port_buffer;
    std::complex<float> * rx_port_buffer;

    // intput/output data ports
    gport port_tx;
    gport port_rx;
    gport port_resamp_tx;
    gport port_resamp_rx;

    // gain
    float tx_gain;              // nominal tx gain
    float rx_gain;              // nominal rx gain
    float rx_gain_correction;   // rx gain correction factor

#if USRPIO_USE_DC_BLOCKER
    // dc blocker
    std::complex<float> m_hat;
    float alpha;
    float beta;
#endif

    // frequency

    // interp/decim rates
    unsigned int tx_interp0, tx_interp1;
    unsigned int rx_decim0, rx_decim1;

    // arbitrary resampling properties/objects
    float rx_resamp_rate;
    float tx_resamp_rate;

    resamp_crcf rx_resamp;
    resamp_crcf tx_resamp;
};

