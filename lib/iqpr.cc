//
// liquid packet radio
//

#include <iostream>
#include <complex>
#include <math.h>
#include <liquid/liquid.h>

#include "iqpr.h"
#include "usrp_io.h"

// callback
void callback(void);

struct iqpr_s {
    unsigned int node_id;           // node identification
    unsigned int net_id;            // network identification

    usrp_io * uio;                  // usrp input/output device

    // framing objects
    flexframegen fg;                // frame generator
    flexframegenprops_s fgprops;    // frame generator properties
    flexframesync fs;               // frame synchronizer
    flexframesyncprops_s fsprops;   // frame synchronizer properties

    // filtering objects
    interp_crcf  mf_interp;         // matched filter interpolator
    resamp2_crcf interp;            // half-band interpolator
    resamp2_crcf decim;             // half-band decimator

    // packetizer objects
    packetizer p_enc;               // packet encoder
    packetizer p_dec;               // packet decoder

    // certain properties
    float gain_tx;                  // transmit gain

    // MAC layer
    int tx_mutex;                   // transmit mutex lock for...
    int ack_timeout;                //
    unsigned char header_tx[8];     // transmit header
    unsigned char header_rx[8];     // receive header

    // buffers
    unsigned int payload_len;       // decoded message length (bytes)
    unsigned int packet_len;        // encoded message length (bytes)
    unsigned int frame_len;         // frame length (complex samples)
    std::complex<float> * mf_buffer;
    std::complex<float> * frame;
    std::complex<float> data_tx[512];

    // extended
    void * nodes;   // list of nodes in network
};

