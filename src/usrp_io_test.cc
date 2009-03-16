//
//
//

#include "usrp_io.h"

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

    // start
    usrp->start_rx(0);
    usrp->start_tx(0);

    // stop
    usrp->stop_rx(0);
    usrp->stop_tx(0);

    // delete usrp object
    delete usrp;
}
