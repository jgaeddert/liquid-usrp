//
//
//

#include <complex>
#include <usrp_standard.h>
#include <pthread.h>
#include <liquid/liquid.h>
#include "db_base.h"

// threading functions
void* usrp_io_tx_process(void * _u);
void* usrp_io_rx_process(void * _u);

class usrp_io
{
    // friend functions allow private access
    friend void* usrp_io_tx_process(void * _u);
    friend void* usrp_io_rx_process(void * _u);

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

    // decimation
    void get_tx_decim(int _channel, int &_decim);
    void get_rx_decim(int _channel, int &_decim);
    void set_tx_decim(int _channel, int _decim);
    void set_rx_decim(int _channel, int _decim);

    // other properties
    void enable_auto_tx(int _channel)   { tx_db0->set_auto_tr(true);  }
    void disable_auto_tx(int _channel)  { tx_db0->set_auto_tr(false); }

    // port handling
    gport get_tx_port(int _channel) { return port_tx; }
    gport get_rx_port(int _channel) { return port_rx; }

protected:
    // initialization methods
    void initialize();

    // gr/usrp objects
    usrp_standard_rx * usrp_rx;
    usrp_standard_tx * usrp_tx;

    // daughterboard
    db_base * rx_db0;
    db_base * rx_db1;
    db_base * tx_db0;
    db_base * tx_db1;

    // flags
    bool use_complex;
    bool rx_active;
    bool tx_active;

    // threads
    pthread_t tx_thread;
    pthread_t rx_thread;

    // user data
    void * tx_userdata;
    void * rx_userdata;

    // internal buffering
    unsigned int tx_buffer_length;
    unsigned int rx_buffer_length;
    short * tx_buffer;
    short * rx_buffer;
    float rx_gain;  // receiver gain correction factor
    float tx_gain;  // transmitter gain correction factor

    // intput/output data ports
    gport port_tx;
    gport port_rx;
};

