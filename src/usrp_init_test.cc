//
//
//

#include <iostream>

#include "usrp_standard.h"
#include "usrp_prims.h"
#include "usrp_dbid.h"

int main() {
    // initialize rx
    usrp_standard_rx * usrp_rx = usrp_standard_rx::make(0,256);

    if (!usrp_rx) {
        std::cerr << "could not create usrp rx" << std::endl;
        exit(0);
    }

    //db_base * rx_db0_control
    int rx_db0 = usrp_rx->daughterboard_id(0);
    int rx_db1 = usrp_rx->daughterboard_id(1);

    std::cout << "rx db slot 0 : " << usrp_dbid_to_string(rx_db0) << std::endl;
    std::cout << "rx db slot 1 : " << usrp_dbid_to_string(rx_db1) << std::endl;

    printf("done.\n");
    return 0;
}

