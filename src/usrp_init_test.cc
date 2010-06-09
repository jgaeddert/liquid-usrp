//
//
//

#include <iostream>
#include <stdio.h>

#include "usrp_standard.h"
#include "usrp_prims.h"
#include "usrp_dbid.h"


//#include "USRP.h"
#include "flex.h"
#include "basic.h"
#include "lf.h"
#include "dbsrx.h"
#include "tvrx.h"

#define USRP_CHANNEL    0
#define USRP_VERSION    3

int main() {
    // initialize rx
#if USRP_VERSION < 3
    usrp_standard_rx * usrp_rx = usrp_standard_rx::make(USRP_CHANNEL,256);
#else
    usrp_standard_rx_sptr usrp_rx = usrp_standard_rx::make(USRP_CHANNEL,256);
#endif

    if (!usrp_rx) {
        std::cerr << "could not create usrp rx" << std::endl;
        exit(0);
    }

    //ossie_db_base * rx_db0_control
    int rx_db0 = usrp_rx->daughterboard_id(0);
    int rx_db1 = usrp_rx->daughterboard_id(1);

    unsigned int buffer_len = 1024;
    short int rx_buffer[buffer_len];
    bool overrun=false;
    float frequency = 462e6;

    // from ossie
#if USRP_VERSION < 3
    ossie_db_base * rx_db0_control;
    //ossie_db_base * rx_db1_control;

    std::cout << "rx db slot 0 : " << usrp_dbid_to_string(rx_db0) << std::endl;
    std::cout << "rx db slot 1 : " << usrp_dbid_to_string(rx_db1) << std::endl;

    if (rx_db0 == USRP_DBID_FLEX_400_RX_MIMO_B) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400_RX_MIMO_B\n");
        rx_db0_control = new db_flex400_rx_mimo_b(usrp_rx,0);
    } else {
        printf("use usrp db flex 400 rx MIMO B\n");
        return 0;
    }

    // set the ddc frequency
    usrp_rx->set_rx_freq(USRP_CHANNEL, 0.0);

    // set the daughterboard frequency
    float db_lo_offset = -8e6;
    float db_lo_freq = 0.0f;
    float db_lo_freq_set = frequency + db_lo_offset;
    rx_db0_control->set_db_freq(db_lo_freq_set, db_lo_freq);
    float ddc_freq = frequency - db_lo_freq;
    usrp_rx->set_rx_freq(USRP_CHANNEL, ddc_freq);

#else
    db_base_sptr rx_db0_control = usrp_rx->db(0)[0];

    std::cout << "rx db slot 0 : " << usrp_dbid_to_string(rx_db0) << std::endl;
    std::cout << "rx db slot 1 : " << usrp_dbid_to_string(rx_db1) << std::endl;

    // tune

    usrp_tune_result result;
    usrp_rx->tune(USRP_CHANNEL, rx_db0_control, frequency, &result);
#if 0
    if (!usrp_rx->tune(USRP_CHANNEL, rx_db0_control, frequency, &result)) {
        fprintf(stderr,"error: tune failed\n");
        exit(1);
    }
#endif

    // print debug info
    printf("usrp rx tune result:\n");
    printf("    baseband freq   :   %12.4e\n", result.baseband_freq);
    printf("    dxc_freq        :   %12.4e\n", result.dxc_freq);
    printf("    residual_freq   :   %12.4e\n", result.residual_freq);
    printf("    inverted        :   %u\n", result.inverted);

#endif
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

