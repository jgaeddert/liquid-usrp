/*
 * Copyright (c) 2007, 2008, 2009, 2010 Joseph Gaeddert
 * Copyright (c) 2007, 2008, 2009, 2010 Virginia Polytechnic
 *                                      Institute & State University
 *
 * This file is part of liquid.
 *
 * liquid is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * liquid is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with liquid.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//
//

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <complex>
#include <liquid/liquid.h>
#include "usrp_io.h"

#define USRP_CHANNEL    (0)

void * tx_handler( void * _port );
void * rx_handler( void * _port );

int main() {
    // options
    float   tx_freq     = 462e6f;
    float   rx_freq     = 462.5625e6f;
    int     tx_interp   = 512;
    int     rx_decim    = 256;

    // create usrp object
    usrp_io * usrp = new usrp_io();

    // set properties
    usrp->set_tx_freq(USRP_CHANNEL, tx_freq);
    usrp->set_tx_interp(tx_interp);
    usrp->set_rx_freq(USRP_CHANNEL, rx_freq);
    usrp->set_rx_decim(rx_decim);
    usrp->enable_auto_tx(USRP_CHANNEL);

    // ports
    gport port_tx = usrp->get_tx_port(USRP_CHANNEL);
    gport port_rx = usrp->get_rx_port(USRP_CHANNEL);

    // threads
    pthread_t tx_thread;
    pthread_t rx_thread;
    pthread_attr_t thread_attr;
    void * status;
    
    // set thread attributes
    pthread_attr_init(&thread_attr);
    pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_JOINABLE);

    // attributes object no longer needed
    pthread_attr_destroy(&thread_attr);

    std::cout << "waiting to start threads..." << std::endl;
    usleep(2000000);

    // create threads
    pthread_create(&tx_thread, &thread_attr, &tx_handler, (void*) port_tx);
    pthread_create(&rx_thread, &thread_attr, &rx_handler, (void*) port_rx);

    // start data transfer
    usrp->start_tx(USRP_CHANNEL);
    usrp->start_rx(USRP_CHANNEL);

    std::cout << "waiting for threads to exit..." << std::endl;

    // join threads
    pthread_join(tx_thread, &status);
    pthread_join(rx_thread, &status);

    // stop
    usrp->stop_rx(USRP_CHANNEL);
    usrp->stop_tx(USRP_CHANNEL);

    printf("main process complete\n");

    // delete usrp object
    delete usrp;
}

void * tx_handler ( void *_port )
{
    gport port = (gport) _port;
 
    // interpolator options
    unsigned int m=4;
    float beta=0.3f;
    unsigned int h_len = 2*2*m + 1;
    float h[h_len];
    design_rrc_filter(2,m,beta,0,h);
    interp_crcf interp = interp_crcf_create(2,h,h_len);

    unsigned int num_symbols=256;
    std::complex<float> s[num_symbols];
    std::complex<float> x[2*num_symbols];
    unsigned int i;
    printf("tx thread running...\n");
    for (unsigned int n=0; n<2000; n++) {
    //while (1) {
        // get data from port
        for (i=0; i<num_symbols; i++) {
            s[i].real() = rand()%2 ? 1.0f : -1.0f;
            s[i].imag() = rand()%2 ? 1.0f : -1.0f;
            interp_crcf_execute(interp, s[i], &x[2*i]);
        }
        gport_produce(port,(void*)x,2*num_symbols);
    }
    std::cout << "done." << std::endl;
    interp_crcf_destroy(interp);
   
    printf("tx_handler finished.\n");
    pthread_exit(0); // exit thread
}


void * rx_handler ( void *_port )
{
    gport p = (gport) _port;

    unsigned int nfft=64;
    std::complex<float> data_rx[512];
    std::complex<float> spectrogram_buffer[nfft];

    asgram sg = asgram_create(spectrogram_buffer,nfft);
    asgram_set_scale(sg,10.0f);
    asgram_set_offset(sg,-20.0f);

    float peakval;
    float peakfreq;
    char ascii[nfft+1];
    ascii[nfft] = '\0'; // append null character to end of string

    for (unsigned int n=0; n<4000; n++) {
        gport_consume(p,(void*)data_rx,512);
        
        // run ascii spectrogram
        if (n%30 == 0) {
            memmove(spectrogram_buffer, data_rx, nfft*sizeof(std::complex<float>));
            asgram_execute(sg, ascii, &peakval, &peakfreq);

            // print the spectrogram
            printf(" > %s < pk%5.1fdB [%5.2f]\n", ascii, peakval, peakfreq);

        }
    }

    printf("rx_handler finished.\n");
    pthread_exit(0); // exit thread
}


