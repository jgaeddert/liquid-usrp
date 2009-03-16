
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

}

usrp_io::~usrp_io()
{
    // TODO: delete usrp_rx and usrp_tx objects
}

// start/stop
void usrp_io::start_rx(int _channel) {}
void usrp_io::start_tx(int _channel) {}
void usrp_io::stop_rx(int _channel) {}
void usrp_io::stop_tx(int _channel) {}

// gain
void usrp_io::get_tx_gain(int _channel, float &_gain) {}
void usrp_io::get_rx_gain(int _channel, float &_gain) {}
void usrp_io::set_tx_gain(int _channel, float _gain) {}
void usrp_io::set_rx_gain(int _channel, float _gain) {}

// frequency
void usrp_io::get_tx_freq(int _channel, float &_freq) {}
void usrp_io::get_rx_freq(int _channel, float &_freq) {}
void usrp_io::set_tx_freq(int _channel, float _freq) {}
void usrp_io::set_rx_freq(int _channel, float _freq) {}

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

}

