//
//
//

#include <usrp_standard.h>
#include "db_base.h"

typedef void* (*tx_callback)(short *_I, short *_Q, unsigned int _n, void * _userdata);
typedef void* (*rx_callback)(short *_I, short *_Q, unsigned int _n, void * _userdata);

class usrp_io
{
public:
    // default constructor
    usrp_io();
    ~usrp_io();

    // start/stop
    void start_tx(int _channel, tx_callback _callback);
    void start_rx(int _channel, rx_callback _callback);
    void stop_tx(int _channel);
    void stop_rx(int _channel);

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

private:
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
};

