//
//
//

#include <iostream>
#include <liquid/liquid.h>

#include "usrp_standard.h"
#include "usrp_prims.h"
#include "usrp_dbid.h"

#define USRP_CHANNEL    0

int main() {
    // initialize rx
    usrp_standard_rx * usrp_rx = usrp_standard_rx::make(USRP_CHANNEL,256);

    if (!usrp_rx) {
        std::cerr << "could not create usrp rx" << std::endl;
        exit(0);
    }

    //db_base * rx_db0_control
    int rx_db0 = usrp_rx->daughterboard_id(0);
    int rx_db1 = usrp_rx->daughterboard_id(1);

    std::cout << "rx db slot 0 : " << usrp_dbid_to_string(rx_db0) << std::endl;
    std::cout << "rx db slot 1 : " << usrp_dbid_to_string(rx_db1) << std::endl;

    if (rx_db0 == USRP_DBID_FLEX_400_RX_MIMO_B) {
        printf("ok!!!\n");
    } else {
        printf("use usrp db flex 400 rx MIMO B\n");
        return 0;
    }

    unsigned int buffer_len = 1024;
    short int rx_buffer[buffer_len];
    bool overrun=false;

    // set the ddc frequency
    usrp_rx->set_rx_freq(USRP_CHANNEL, 0.0);

    // set the daughterboard frequency
    //rx_db0->xxx;

    // start
    usrp_rx->start();

    unsigned int i, j;
    for (i=0; i<10; i++) {
        usrp_rx->read(rx_buffer, buffer_len, &overrun);

        // print several values of buffer
        printf("%4u: ", i);
        for (j=0; j<8; j++)
            printf("%8d, ", rx_buffer[j]);
        printf("\n");
    }

    usrp_rx->stop();

    printf("done.\n");
    return 0;
}

