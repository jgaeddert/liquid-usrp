
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

    tx_buffer = new short[2*tx_buffer_length];
    rx_buffer = new short[2*tx_buffer_length];

    port_tx = gport_create(2048, sizeof(std::complex<float>));
    port_rx = gport_create(2048, sizeof(std::complex<float>));

    // gain levels
    rx_gain = 1.0f / 8000.0f;
    tx_gain = 8000.0f;
}

usrp_io::~usrp_io()
{
    // TODO: delete usrp_rx and usrp_tx objects

    // destroy ports
    gport_destroy(port_tx);
    gport_destroy(port_rx);

    delete [] tx_buffer;
    delete [] rx_buffer;
}

// start/stop
//void usrp_io::start_tx(int _channel, usrp_tx_callback _callback, void * _userdata)
void usrp_io::start_tx(int _channel)
{
    if (_channel != 0) {
        std::cerr << "error: usrp_io::start_tx(), only channel 0 currently supported" << std::endl;
        throw 0;
    } else if (tx_active) {
        std::cerr << "error: usrp_io::start_tx(), tx active" << std::endl;
        throw 0;
    }
    //tx_callback0 = _callback;
    tx_active = true;
    //tx_userdata = _userdata;
    pthread_create(&tx_thread, NULL, usrp_io_tx_process, this);
}

//void usrp_io::start_rx(int _channel, usrp_rx_callback _callback, void * _userdata)
void usrp_io::start_rx(int _channel)
{
    if (_channel != 0) {
        std::cerr << "error: usrp_io::start_rx(), only channel 0 currently supported" << std::endl;
        throw 0;
    } else if (rx_active) {
        std::cerr << "error: usrp_io::start_rx(), rx active" << std::endl;
        throw 0;
    }
    //rx_callback0 = _callback;
    rx_active = true;
    //rx_userdata = _userdata;
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

// decimation
void usrp_io::get_tx_decim(int _channel, int &_decim) {}
void usrp_io::get_rx_decim(int _channel, int &_decim) {}
void usrp_io::set_tx_decim(int _channel, int _decim) {}
void usrp_io::set_rx_decim(int _channel, int _decim) {}

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
}

// threading functions
void* usrp_io_tx_process(void * _u)
{
    std::cout << "usrp_io_tx_process() invoked" << std::endl;
    usrp_io * usrp = (usrp_io*) _u;
    //void * userdata = usrp->tx_userdata;

    // local variables
    int rc;
    bool underrun;

    // start data transfer
    usrp->usrp_tx->start();

    std::complex<float> * data;

    while (usrp->tx_active) {
        // invoke callback
        //usrp->tx_callback0(usrp->tx_buffer, usrp->tx_buffer_length, userdata);
        // wait for data
        data = (std::complex<float>*) gport_consumer_lock(usrp->port_tx,512);

        // convert to short
        for (unsigned int i=0; i<usrp->tx_buffer_length; i++) {
            usrp->tx_buffer[2*i+0] = (short)(data[i].real() * usrp->tx_gain);
            usrp->tx_buffer[2*i+1] = (short)(data[i].imag() * usrp->tx_gain);
        }

        // write data
        rc = usrp->usrp_tx->write(usrp->tx_buffer, 2*usrp->tx_buffer_length, &underrun);

        if (rc < 0) {
            std::cerr << "error: usrp_io_tx_process(), tx error" << std::endl;
            throw 0;
        } else if (rc != (int)(2*usrp->tx_buffer_length) ) {
            std::cerr << "warning: usrp_io_tx_process(), usrp attempted to write "
                      << usrp->tx_buffer_length << " values ("
                      << rc << " actually written)" << std::endl;
        }

        if (underrun)
            std::cerr << "underrun" << std::endl;

        // unlock data
        gport_consumer_unlock(usrp->port_tx,512);
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
    //void * userdata = usrp->rx_userdata;

    // local variables
    int rc;
    bool overrun;
    std::complex<float> * data;

    // start data transfer
    usrp->usrp_rx->start();

    while (usrp->rx_active) {
        // read data
        rc = usrp->usrp_rx->read(usrp->rx_buffer, 2*usrp->rx_buffer_length, &overrun);

        if (rc < 0) {
            std::cerr << "error: usrp_io_rx_process(), rx error ("
                      << rc << ")" << std::endl;
            throw 0;
        } else if (rc != (int)(2*usrp->rx_buffer_length) ) {
            std::cerr << "warning: usrp_io_rx_process(), usrp attempted to write "
                      << usrp->rx_buffer_length << " values ("
                      << rc << " actually written)" << std::endl;
        }

        if (overrun)
            std::cerr << "overrun" << std::endl;

        // invoke callback
        //usrp->rx_callback0(usrp->rx_buffer, usrp->rx_buffer_length, userdata);

        data = (std::complex<float>*) gport_producer_lock(usrp->port_rx,512);

        // convert to complex float
        for (unsigned int i=0; i<usrp->tx_buffer_length; i++) {
            data[i].real() = (float)(usrp->rx_buffer[2*i+0]) * usrp->rx_gain;
            data[i].imag() = (float)(usrp->rx_buffer[2*i+1]) * usrp->rx_gain;
        }
        gport_producer_unlock(usrp->port_rx,512);

    }

    // stop data transfer
    usrp->usrp_rx->stop();

    std::cout << "usrp_io_rx_process() terminating" << std::endl;
    pthread_exit(NULL);
}


