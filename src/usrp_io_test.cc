//
//
//

#include <iostream>
#include "usrp_io.h"

void* tx_nco_callback(short *_iq_data, unsigned int _n, void * _userdata);
void* rx_display_callback(short *_iq_data, unsigned int _n, void * _userdata);

int main() {
    // options
    float   tx_freq     = 462e6f;
    float   rx_freq     = 427e6f;
    int     tx_decim    = 256;
    int     rx_decim    = 256;

    // create usrp object
    usrp_io * usrp = new usrp_io();

    // set properties
    usrp->set_tx_freq(0, tx_freq);
    usrp->set_rx_freq(0, rx_freq);
    usrp->set_tx_decim(0, tx_decim);
    usrp->set_rx_decim(0, rx_decim);
    usrp->enable_auto_tx(0);

    // start
    usrp->start_tx(0,tx_nco_callback,NULL);
    usrp->start_rx(0,rx_display_callback,NULL);

    // process data, wait, configure properties
    std::cout << "waiting..." << std::endl;
    usleep(4000000);
    std::cout << "done." << std::endl;

    // stop
    usrp->stop_rx(0);
    usrp->stop_tx(0);

    // delete usrp object
    delete usrp;
}

void* tx_nco_callback(short *_iq_data, unsigned int _n, void * _userdata)
{
    std::cout << "tx_nco_callback() invoked" << std::endl;
    return NULL;
}

void* rx_display_callback(short *_iq_data, unsigned int _n, void * _userdata)
{
    std::cout << "rx_display_callback() invoked" << std::endl;
    return NULL;
}

