//
//
//

#include <iostream>
#include <math.h>
#include <complex>
#include <liquid/liquid.h>
#include "usrp_io.h"

void* tx_nco_callback(short *_iq_data, unsigned int _n, void * _userdata);
void* rx_display_callback(short *_iq_data, unsigned int _n, void * _userdata);

std::complex<float> x[512];
//liquid_float_complex x[512];
interp_crcf interp;

int main() {
    // options
    float   tx_freq     = 462e6f;
    float   rx_freq     = 485e6f;
    int     tx_decim    = 256;
    int     rx_decim    = 256;

    unsigned int m=2;
    float beta=0.3f;
    unsigned int h_len = 2*2*m + 1;
    float h[h_len];
    design_rrc_filter(2,m,beta,0,h);
    
    interp = interp_crcf_create(2,h,h_len);

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
    usleep(50000000);
    std::cout << "done." << std::endl;

    // stop
    usrp->stop_rx(0);
    usrp->stop_tx(0);

    interp_crcf_destroy(interp);

    // delete usrp object
    delete usrp;
}

void* tx_nco_callback(short *_iq_data, unsigned int _n, void * _userdata)
{
    //std::cout << "tx_nco_callback() invoked" << std::endl;

    unsigned int i;
    std::complex<float> s;
    //liquid_float_complex s;

    std::complex<float> d[2];
    //liquid_float_complex d[2];
    for (i=0; i<_n/2; i+=2) {
        s.real() = rand()%2 ? 1.0f : -1.0f;
        s.imag() = rand()%2 ? 1.0f : -1.0f;
        //s[0] = rand()%2 ? 1.0f : -1.0f;
        //s[1] = rand()%2 ? 1.0f : -1.0f;
        interp_crcf_execute(interp, s, &x[i]);

        //x[i+0] = s;
        //x[i+1] = s;
    }

    for (i=0; i<_n; i+=2) {
        _iq_data[i+0] = (short) (4000 * (x[i].real()));
        _iq_data[i+1] = (short) (4000 * (x[i].imag()));
    }
    return NULL;
}

void* rx_display_callback(short *_iq_data, unsigned int _n, void * _userdata)
{
    //std::cout << "rx_display_callback() invoked" << std::endl;
    unsigned int i;
    short I,Q;
    float e=0.0f;
    for (i=0; i<_n; i+=2) {
        I = _iq_data[i+0];
        Q = _iq_data[i+1];
        e += fabsf(I) + fabsf(Q);
    }
    e /= _n;
    std::cout << "energy: " << e << std::endl;
    return NULL;
}

