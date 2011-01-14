/*
 * Copyright (c) 2011 Joseph Gaeddert
 * Copyright (c) 2011 Virginia Polytechnic Institute & State University
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
// crdemo.cc
//
// cognitive radio demo: ping basic data packets back and forth
//

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <getopt.h>
#include <pthread.h>
#include <sys/time.h>
#include <complex>

#include "config.h"
#if HAVE_LIBLIQUIDRM
#include <liquid/liquidrm.h>
#endif
#include <liquid/liquid.h>

#include "usrp_io.h"
#include "iqpr.h"

#define USRP_CHANNEL        (0)

#define NODE_MASTER         (0)
#define NODE_SLAVE          (1)

void usage() {
    printf("crdemo usage:\n");
    printf("  u,h   :   usage/help\n");
    printf("  f     :   frequency [Hz], default: 462 MHz\n");
    printf("  b     :   bandwidth [Hz], default: 100 kHz\n");
    printf("  n     :   number of packets, default: 1000\n");
    printf("  a     :   number of tx attempts (master), default: 100\n");
    printf("  m/s   :   designate node as master/slave, default: slave\n");
    printf("  v/q   :   set verbose/quiet mode, default: verbose\n");
}

int main (int argc, char **argv) {
    // options
    float frequency = 462e6f;
    float symbolrate = 100e3f;
    unsigned int num_packets = 1000;
    unsigned int max_num_attempts = 100;    // maximum number of tx attempts
    unsigned int node_type = NODE_MASTER;
    unsigned int node_id = 15;              // node id in [0,15]
    unsigned int rssi_samples = 128;        // number of samples with which to estimate rssi
    float rssi_clear_threshold = -35.0f;    // rssi threshold to determine if channel is 'clear'
    unsigned int mac_timeout = 5;           // number of 'clear' flags before transmission
    //unsigned int mac_timeout_backoff = 5;   // random backoff
    float tx_gain = 0.9f;                   // transmit gain
    int verbose = 1;

    //
    int d;
    while ((d = getopt(argc,argv,"uhf:b:n:a:msvq")) != EOF) {
        switch (d) {
        case 'u':
        case 'h': usage();                          return 0;
        case 'f': frequency = atof(optarg);         break;
        case 'b': symbolrate = atof(optarg);        break;
        case 'n': num_packets = atoi(optarg);       break;
        case 'a': max_num_attempts = atoi(optarg);  break;
        case 'm': node_type = NODE_MASTER;          break;
        case 's': node_type = NODE_SLAVE;           break;
        case 'v': verbose = 1;                      break;
        case 'q': verbose = 0;                      break;
        default:
            fprintf(stderr,"error: %s, unsupported option\n", argv[0]);
            exit(1);
        }
    }

    // create usrp object
    usrp_io * usrp = new usrp_io();

    // set properties
    usrp->set_tx_freq(USRP_CHANNEL, frequency);
    usrp->set_tx_samplerate(2*symbolrate);
    usrp->set_rx_freq(USRP_CHANNEL, frequency);
    usrp->set_rx_samplerate(2*symbolrate);
    usrp->enable_auto_tx(USRP_CHANNEL);

    if (verbose) usrp->enable_verbose();
    else         usrp->disable_verbose();

    // initialize iqpr structure
    iqpr q = iqpr_create(node_id,
                         usrp->get_tx_port(USRP_CHANNEL),
                         usrp->get_rx_port(USRP_CHANNEL));

    // iqpr_setverbose(q, verbose);

    // set transmit gain
    iqpr_settxgain(q,tx_gain);
    float tx_gain_dB = 20*logf(tx_gain);

    // sleep for a small time before starting tx/rx processes
    usleep(1000000);

    // start data transfer
    usrp->start_tx(USRP_CHANNEL);
    usrp->start_rx(USRP_CHANNEL);

    // TODO : start timer

    //unsigned int i;
    unsigned int pid;
    unsigned int j;
    unsigned int n;
    unsigned int num_attempts = 0;
    unsigned int t=0;   // number of transmitted packets

    // frame headers
    iqprheader_s tx_header; // transmitted frame header
    iqprheader_s rx_header; // received frame header

    // received frame statistics (SNR, rssi)
    framesyncstats_s stats;

    // parameters
    modulation_scheme ms = MOD_PSK;
    unsigned int bps = 2;
    fec_scheme fec0 = FEC_NONE;
    fec_scheme fec1 = FEC_NONE;


    if (node_type == NODE_MASTER) {
        // 
        // MASTER NODE
        //
        unsigned int payload_len = 1024;
        unsigned char payload[payload_len];

        unsigned char * rx_payload = NULL;
        unsigned int rx_payload_len;
        unsigned int packet_found;
        float master_path_loss_dB;  // master path loss [dB]
        float slave_cpuload;        // slave node cpu load

        int ack_received;

        for (pid=0; pid<num_packets; pid++) {

            // initialize payload to random data
            for (n=0; n<payload_len; n++)
                payload[n] = rand() % 256;

            ack_received = 0;
            num_attempts = 0;
            do {
                num_attempts++;
                t++;

                // wait for clear signal (five clean reports in a row)
                j = mac_timeout;
                while (j) {
                    float rssi = iqpr_mac_getrssi(q,rssi_samples);
                    int clear = rssi < rssi_clear_threshold;
                    //if (verbose) printf("  rssi : %12.8f dB %c\n", rssi, clear ? ' ' : '*');

                    if (clear) j--;
                    else       j = mac_timeout;
                }

#if 0
                // time-varying gain
                tx_gain_dB = -25.0f*(0.5f - 0.5f*sinf(2*M_PI*(float)t / 200.0f));
#else
                // constant gain
                tx_gain_dB =  -3.0f;
#endif
                
                tx_gain = powf(10.0f, tx_gain_dB/10.0f);
                iqpr_settxgain(q,tx_gain);

                // initialize header
                tx_header.pid           = pid;
                tx_header.packet_type   = IQPR_PACKET_TYPE_DATA;
                tx_header.node_src      = node_id;
                tx_header.node_dst      = 0;
                tx_header.userdata[0]   = (unsigned char)(10.0f*(tx_gain_dB + 25.0f));
                tx_header.userdata[1]   = 0;
                tx_header.userdata[2]   = 0;

                // transmit packet
                printf("transmitting packet %6u/%6u (attempt %4u/%4u) %c\n",
                        pid, num_packets, num_attempts, max_num_attempts,
                        num_attempts > 1 ? '*' : ' ');
                iqpr_txpacket(q,&tx_header,payload,payload_len,ms,bps,fec0,fec1);

                // wait for acknowledgement (minimum timeout is about 3)
                // TODO : increase timeout based on transmitted packet length
                for (j=0; j<5; j++) {
#if 0
                    ack_received = iqpr_wait_for_ack(q, pid, &rx_header, &stats);
#else
                    packet_found = iqpr_wait_for_packet(q,
                                                        &rx_payload,
                                                        &rx_payload_len,
                                                        &rx_header,
                                                        &stats);
                
                    if (packet_found &&
                        rx_header.userdata[1] == IQPR_PACKET_TYPE_ACK &&
                        rx_header.pid         == tx_header.pid)
                    {
                        ack_received = 1;

                        // decode path loss
                        master_path_loss_dB = rx_header.userdata[0] / 4.0f;
                        slave_cpuload = rx_header.userdata[2] / 250.0f;
                        if (verbose) {
                            printf("ack received on packet [%4u], path loss=%8.2fdB, cpuload=%8.2f %%\n",
                                    tx_header.pid,
                                    master_path_loss_dB,
                                    slave_cpuload * 100.0f);
                        }
                    }
#endif

                    if (ack_received)
                        break;
                }
            } while (!ack_received && (num_attempts < max_num_attempts) );

            if (num_attempts == max_num_attempts) {
                printf("transmitter reached maximum number of attemts; bailing\n");
                break;
            }
        }
    } else {
        // 
        // SLAVE NODE
        //
        unsigned char * payload = NULL;
        unsigned int payload_len;
        unsigned int packet_found;
#if HAVE_LIBLIQUIDRM
        // create/initialize resource-monitoring daemon
        rmdaemon rmd = rmdaemon_create();
        rmdaemon_start(rmd);
        double runtime;
        double cpuload = 0.15f;
#endif

        pid = 0;

        do {
            // wait for data packet
            do {
                // attempt to receive data packet
                packet_found = iqpr_wait_for_packet(q,
                                                    &payload,
                                                    &payload_len,
                                                    &rx_header,
                                                    &stats);
#if HAVE_LIBLIQUIDRM
                // compute cpu load
                runtime = rmdaemon_gettime(rmd);
                if ( runtime > 0.5f ) {
                    cpuload = 0.8f*cpuload + 0.2f*rmdaemon_getcpuload(rmd);
                    rmdaemon_resettimer(rmd);
                    if (verbose) printf("  cpuload : %f\n", cpuload);
                }
#endif
            } while (!packet_found);

            printf("  crdemo received %4u data bytes on packet [%4u], {%3u %3u %3u}\n",
                    payload_len,
                    rx_header.pid,
                    (unsigned int) rx_header.userdata[0],
                    (unsigned int) rx_header.userdata[1],
                    (unsigned int) rx_header.userdata[2]);

            // print received frame statistics
            if (verbose) framesyncstats_print(&stats);

#if 0
            // transmit acknowledgement
            iqpr_txack(q, rx_header.pid);
#else
            // send our own, home-brewed acknowledgement

            // compute path loss
            float master_tx_gain_dB = (float)(rx_header.userdata[0])*0.1f - 25.0f;
            float path_loss_dB = master_tx_gain_dB - stats.rssi;
            //printf("  path loss : %12.8f dB\n", path_loss_dB);

            // initialize header
            tx_header.pid           = rx_header.pid;
            tx_header.packet_type   = IQPR_PACKET_TYPE_DATA;
            tx_header.node_src      = node_id;
            tx_header.node_dst      = rx_header.node_src;
            tx_header.userdata[0]   = (unsigned char)(4*path_loss_dB);
            tx_header.userdata[1]   = IQPR_PACKET_TYPE_ACK;
#if HAVE_LIBLIQUIDRM
            if (cpuload > 1.0) cpuload = 1.0;
            tx_header.userdata[2]   = (unsigned char) (cpuload * 250);
#else
            tx_header.userdata[2]   = 0;
#endif

            // transmit ACK packet
            iqpr_txpacket(q,&tx_header,NULL,0,MOD_BPSK,1,FEC_NONE,FEC_NONE);
#endif

            pid = rx_header.pid;

        } while (pid != num_packets-1);

#if HAVE_LIBLIQUIDRM
        // stop/destroy resource-monitoring daemon
        rmdaemon_stop(rmd);
        rmdaemon_destroy(rmd);
#endif
    }

    // TODO : stop timer

    // sleep for a small time before stopping tx/rx processes
    usleep(100000);

    // stop data transfer
    usrp->stop_rx(USRP_CHANNEL);
    usrp->stop_tx(USRP_CHANNEL);

    printf("main process complete\n");

    // delete usrp object
    delete usrp;

    // destroy main data object
    iqpr_destroy(q);

    return 0;
}

