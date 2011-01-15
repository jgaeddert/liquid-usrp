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
#include <liquid/liquidce.h>
#include <liquid/liquid.h>

#include "usrp_io.h"
#include "iqpr.h"

#define OUTPUT_FILENAME     "crdemo.dat"

#define USRP_CHANNEL        (0)

#define NODE_MASTER         (0)
#define NODE_SLAVE          (1)

#define PARAM_MOD_SCHEME    (0)
#define PARAM_FEC0          (1)
#define PARAM_FEC1          (2)
#define PARAM_PAYLOAD       (3)
#define PARAM_TXGAIN        (4)

#define OBSERVABLE_PATHLOSS (0)

#define METRIC_THROUGHPUT   (0)
#define METRIC_TXPOWER      (1)
#define METRIC_COMPLEXITY   (2)

#define NUM_MOD_SCHEMES     (7)

void usage() {
    printf("crdemo usage:\n");
    printf("  u,h   :   usage/help\n");
    printf("  f     :   frequency [Hz], default: 462 MHz\n");
    printf("  b     :   bandwidth [Hz], default: 100 kHz\n");
    printf("  n     :   number of packets, default: 1000\n");
    printf("  a     :   number of tx attempts (master), default: 100\n");
    printf("  N     :   number of engine adaptations (master), default: 100\n");
    printf("  m/s   :   designate node as master/slave, default: slave\n");
    printf("  v/q   :   set verbose/quiet mode, default: verbose\n");
}

void parameter_set_mod_scheme(parameter _p,
                              modulation_scheme _ms,
                              unsigned int _bps);

void parameter_get_mod_scheme(parameter _p,
                              modulation_scheme * _ms,
                              unsigned int * _bps);

void parameter_set_fec_scheme(parameter _p, fec_scheme _fs);

void parameter_get_fec_scheme(parameter _p, fec_scheme * _fs);

void mutate_parameters(modulation_scheme * _ms,
                       unsigned int * _bps,
                       fec_scheme * _fec0,
                       fec_scheme * _fec1,
                       unsigned int * _payload_len,
                       float * _tx_gain_dB);

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
    float channel_gain_dB = 0.0f;           // random time-varying channel gain [dB]
    int verbose = 1;
    unsigned int max_num_adaptations = 100; // number of adaptations to run before bailing

    // engine metrics (master node only)
    float ce_timeout = 1.0f;                // timeout before executing cognition cycle
    unsigned int num_bytes_through=0;       // total number of bytes through channel
    unsigned int num_packets_tx=0;          // total number of packets transmitted
    unsigned int num_packets_rx=0;          // total number of packets received
    float throughput=0;                     // average throughput
    //float spectral_efficiency=0;            // average spectral efficiency
    float average_slave_cpuload=0;          // average slave cpuload
    float average_pathloss=0;               // average pathloss

    //
    int d;
    while ((d = getopt(argc,argv,"uhf:b:n:a:N:msvq")) != EOF) {
        switch (d) {
        case 'u':
        case 'h': usage();                          return 0;
        case 'f': frequency = atof(optarg);         break;
        case 'b': symbolrate = atof(optarg);        break;
        case 'n': num_packets = atoi(optarg);       break;
        case 'a': max_num_attempts = atoi(optarg);  break;
        case 'N': max_num_adaptations=atoi(optarg); break;
        case 'm': node_type = NODE_MASTER;          break;
        case 's': node_type = NODE_SLAVE;           break;
        case 'v': verbose = 1;                      break;
        case 'q': verbose = 0;                      break;
        default:
            fprintf(stderr,"error: %s, unsupported option\n", argv[0]);
            exit(1);
        }
    }

    // create parameters, observables, metrics
    unsigned int num_parameters = 5;
    parameter p[num_parameters];
    p[PARAM_MOD_SCHEME] = parameter_create_discrete("mod-scheme", 7, PARAMETER_UNORDERED);
    p[PARAM_FEC0]       = parameter_create_discrete("fec0", LIQUID_NUM_FEC_SCHEMES, PARAMETER_UNORDERED);
    p[PARAM_FEC1]       = parameter_create_discrete("fec1", LIQUID_NUM_FEC_SCHEMES, PARAMETER_UNORDERED);
    p[PARAM_PAYLOAD]    = parameter_create_discrete("payload-length", 1024, PARAMETER_ORDERED);
    p[PARAM_TXGAIN]     = parameter_create_continuous("tx-gain-dB", -25.0f, 0.0f, PARAMETER_ORDERED);

    unsigned int num_observables = 1;
    observable o[num_observables];
    o[OBSERVABLE_PATHLOSS] = observable_create("path-loss-dB");

    unsigned int num_metrics = 3;
    metric m[num_metrics];
    m[METRIC_THROUGHPUT]    = metric_create("throughput-kbps", METRIC_MAXIMIZE, 80.0f, 0.5f, 0.5f);
    m[METRIC_TXPOWER]       = metric_create("tx-power", METRIC_MINIMIZE, 0.05, 0.1f, 0.3f);
    m[METRIC_COMPLEXITY]    = metric_create("complexity", METRIC_MINIMIZE, 25.0f, 0.25f, 0.2f);

    // create engine
    ce engine = ce_create(p, num_parameters,
                          o, num_observables,
                          m, num_metrics);
    ce_print(engine);
    float zeta = 0.5f;

    // create usrp object
    usrp_io * usrp = new usrp_io();

    // set properties
    usrp->set_tx_freq(USRP_CHANNEL, frequency);
    usrp->set_tx_samplerate(2*symbolrate);
    usrp->set_rx_freq(USRP_CHANNEL, frequency);
    usrp->set_rx_samplerate(2*symbolrate);
    usrp->enable_auto_tx(USRP_CHANNEL);

    usrp->disable_verbose();

    // initialize iqpr structure
    iqpr q = iqpr_create(node_id,
                         usrp->get_tx_port(USRP_CHANNEL),
                         usrp->get_rx_port(USRP_CHANNEL));

    // iqpr_setverbose(q, verbose);

    // set transmit gain
    iqpr_settxgain(q,tx_gain);

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
    unsigned int num_adaptations = 0;
    unsigned int t=0;   // total number of transmitted packets
    int continue_running = 1;

    // frame headers
    iqprheader_s tx_header; // transmitted frame header
    iqprheader_s rx_header; // received frame header

    // received frame statistics (SNR, rssi)
    framesyncstats_s stats;

    // parameters
    modulation_scheme ms = MOD_QPSK;
    unsigned int bps = 1;
    fec_scheme fec0 = FEC_HAMMING74;
    fec_scheme fec1 = FEC_NONE;
    unsigned int payload_len = 200;
    unsigned char payload[1024];
    float tx_gain_dB = 20*logf(tx_gain);

    // initialize parameters
    parameter_set_mod_scheme(p[PARAM_MOD_SCHEME], ms, bps);
    parameter_set_fec_scheme(p[PARAM_FEC0], fec0);
    parameter_set_fec_scheme(p[PARAM_FEC1], fec1);
    parameter_set_discrete_value(p[PARAM_PAYLOAD], payload_len);
    parameter_set_continuous_value(p[PARAM_TXGAIN], tx_gain_dB);

#if HAVE_LIBLIQUIDRM
    // create/initialize resource-monitoring daemon
    rmdaemon rmd = rmdaemon_create();
    rmdaemon_start(rmd);
    double runtime;
    double cpuload = 0.15f;
#endif

    if (node_type == NODE_MASTER) {
        // 
        // MASTER NODE
        //
        unsigned char * rx_payload = NULL;
        unsigned int rx_payload_len;
        unsigned int packet_found;
        float master_path_loss_dB;  // master path loss [dB]
        float slave_cpuload;        // slave node cpu load
        FILE * fid = fopen(OUTPUT_FILENAME, "w");
        if (!fid) {
            fprintf(stderr,"error: could not open '%s' for writing\n", OUTPUT_FILENAME);
            exit(1);
        }
        fprintf(fid,"# %5s %8s %4s %8s %8s %8s %12s %12s %6s %6s %12s %12s %12s\n",
                "epoch", "mod", "bps", "fec0", "fec1", "payload", "gain [dB]",
                "pathloss[dB]", "num rx", "num tx", "kbps", "cpuload (%)",
                "utility");

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

#if HAVE_LIBLIQUIDRM
                // check timer, execute cognition cycle
                runtime = rmdaemon_gettime(rmd);
                if ( runtime > ce_timeout ) {
                    rmdaemon_resettimer(rmd);

                    // compute statistics
                    throughput = num_bytes_through * 8.0f / runtime;
                    //spectral_efficiency = throughput / symbolrate;

                    ce_casedatabase_print(engine);
                    ce_print(engine);

                    // save parameters/observables/metrics
                    parameter_set_mod_scheme(p[PARAM_MOD_SCHEME], ms, bps);
                    parameter_set_fec_scheme(p[PARAM_FEC0], fec0);
                    parameter_set_fec_scheme(p[PARAM_FEC1], fec1);
                    parameter_set_discrete_value(p[PARAM_PAYLOAD], payload_len);
                    parameter_set_continuous_value(p[PARAM_TXGAIN], tx_gain_dB);
                    // 
                    observable_set_value(o[OBSERVABLE_PATHLOSS], average_pathloss);
                    //
                    metric_set_value(m[METRIC_THROUGHPUT], throughput * 1e-3f);
                    metric_set_value(m[METRIC_TXPOWER], powf(10.0f, tx_gain_dB/10.0f));
                    metric_set_value(m[METRIC_COMPLEXITY], average_slave_cpuload*100.0f);

                    // compute utility
                    float u_global = expf(metric_get_weighted_utility(m[METRIC_THROUGHPUT]) +
                                          metric_get_weighted_utility(m[METRIC_TXPOWER]) +
                                          metric_get_weighted_utility(m[METRIC_COMPLEXITY]) );

                    printf("engine: {%-6s(%1u b/s),%6s,%6s,%4u,%6.2f}, PL=%6.2fdB, p[%4u/%4u] %6.2f kbps, cpu: %5.2f%% %6.3f\n",
                            modulation_scheme_str[ms][0],
                            bps,
                            fec_scheme_str[fec0][0],
                            fec_scheme_str[fec1][0],
                            payload_len,
                            tx_gain_dB,
                            average_pathloss,
                            num_packets_rx,
                            num_packets_tx,
                            throughput * 1e-3f,
                            //spectral_efficiency,
                            average_slave_cpuload*100.0f,
                            u_global);

                    // write results to file
                    fprintf(fid,"  %5u %8u %4u %8u %8u %8u %12.6f %12.6f %6u %6u %12.4f %12.4f %12.8f\n",
                            num_adaptations,
                            ms, //modulation_scheme_str[ms][0],
                            bps,
                            fec0, //fec_scheme_str[fec0][0],
                            fec1, //fec_scheme_str[fec1][0],
                            payload_len,
                            tx_gain_dB,
                            average_pathloss,
                            num_packets_rx,
                            num_packets_tx,
                            throughput * 1e-3f,
                            //spectral_efficiency,
                            average_slave_cpuload*100.0f,
                            u_global);
                    // save result
                    ce_retain(engine, p, o, m);

                    // engine adaptation
                    ce_search(engine, o, zeta, p);

                    // prune database...
                    if (ce_casedatabase_get_num_entries(engine) > 50)
                        ce_casedatabase_prune(engine, 0.01f);

                    // save parameter set
                    parameter_get_mod_scheme(p[PARAM_MOD_SCHEME], &ms, &bps);
                    parameter_get_fec_scheme(p[PARAM_FEC0], &fec0);
                    parameter_get_fec_scheme(p[PARAM_FEC1], &fec1);
                    payload_len = parameter_get_discrete_value(p[PARAM_PAYLOAD]);
                    tx_gain_dB = parameter_get_continuous_value(p[PARAM_TXGAIN]);
                    
                    // randomize/mutate
                    mutate_parameters(&ms, &bps, &fec0, &fec1, &payload_len, &tx_gain_dB);

                    // reset counters, etc.
                    num_bytes_through = 0;
                    num_packets_tx = 0;
                    num_packets_rx = 0;
                    average_pathloss = 30.0f;
                    average_slave_cpuload = 1.0f;

                    num_adaptations++;
                    if (num_adaptations == max_num_adaptations)
                        continue_running = 0;

                }
#endif

                // wait for clear signal (five clean reports in a row)
                j = mac_timeout;
                while (j && continue_running) {
                    float rssi = iqpr_mac_getrssi(q,rssi_samples);
                    int clear = rssi < rssi_clear_threshold;
                    //if (verbose) printf("  rssi : %12.8f dB %c\n", rssi, clear ? ' ' : '*');

                    if (clear) j--;
                    else       j = mac_timeout;
                }

                // time-varying channel gain
                channel_gain_dB = -15.0f*(0.5f - 0.5f*sinf(2*M_PI*(float)t / 800.0f));
                //channel_gain_dB = 0.0f;

                tx_gain = powf(10.0f, (tx_gain_dB + channel_gain_dB)/10.0f);
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
#if 0
                printf("transmitting packet %6u/%6u (attempt %4u/%4u) %c\n",
                        pid, num_packets, num_attempts, max_num_attempts,
                        num_attempts > 1 ? '*' : ' ');
#endif
                iqpr_txpacket(q,&tx_header,payload,payload_len,ms,bps,fec0,fec1);
                num_packets_tx++;

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
                        num_packets_rx++;
                        num_bytes_through += payload_len;

                        // decode path loss
                        master_path_loss_dB = rx_header.userdata[0] / 4.0f;
                        slave_cpuload = rx_header.userdata[2] / 250.0f;
                        if (verbose && 0) {
                            printf("ack received on packet [%4u], path loss=%8.2fdB, cpuload=%8.2f %%\n",
                                    tx_header.pid,
                                    master_path_loss_dB,
                                    slave_cpuload * 100.0f);
                        }

                        if (num_packets_rx==1) {
                            average_slave_cpuload = slave_cpuload;
                            average_pathloss = master_path_loss_dB;
                        } else {
                            average_slave_cpuload = 0.5f*average_slave_cpuload + 0.5f*slave_cpuload;
                            average_pathloss = 0.5f*average_pathloss + 0.5f*master_path_loss_dB;
                        }
                    }
#endif

                    if (ack_received)
                        break;
                }
            } while (!ack_received && (num_attempts < max_num_attempts) && continue_running);

            if (num_attempts == max_num_attempts) {
                printf("transmitter reached maximum number of attemts; bailing\n");
                break;
            }

            if (!continue_running) break;
        }
        fclose(fid);
        printf("results written to '%s'\n", OUTPUT_FILENAME);
    } else {
        // 
        // SLAVE NODE
        //
        unsigned char * payload = NULL;
        unsigned int payload_len;
        unsigned int packet_found;
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

    // destroy parameters, observables, metrics
    unsigned int i;
    for (i=0; i<num_parameters; i++)    parameter_destroy(p[i]);
    for (i=0; i<num_observables; i++)   observable_destroy(o[i]);
    for (i=0; i<num_metrics; i++)       metric_destroy(m[i]);

    // destroy engine
    ce_destroy(engine);

    return 0;
}

void parameter_set_mod_scheme(parameter _p,
                              modulation_scheme _ms,
                              unsigned int _bps)
{
    if (_ms == MOD_BPSK)
        parameter_set_discrete_value(_p, 0);
    else if (_ms == MOD_QPSK)
        parameter_set_discrete_value(_p, 1);
    else if (_ms == MOD_PSK && _bps == 3)
        parameter_set_discrete_value(_p, 2);
    else if (_ms == MOD_PSK && _bps == 4)
        parameter_set_discrete_value(_p, 3);
    else if (_ms == MOD_QAM && _bps == 4)
        parameter_set_discrete_value(_p, 4);
    else if (_ms == MOD_QAM && _bps == 5)
        parameter_set_discrete_value(_p, 5);
    else if (_ms == MOD_QAM && _bps == 6)
        parameter_set_discrete_value(_p, 6);
    else {
        fprintf(stderr,"warning: parameter_set_mod_scheme(), invalid configuration (setting to BPSK)\n");
        fprintf(stderr,"         %s, %u\n", modulation_scheme_str[_ms][1], _bps);
        parameter_set_mod_scheme(_p, MOD_BPSK, 1);
    }
}

void parameter_get_mod_scheme(parameter _p,
                              modulation_scheme * _ms,
                              unsigned int * _bps)
{
    unsigned int v = parameter_get_discrete_value(_p);
    switch (v) {
    case 0: *_ms = MOD_BPSK;    *_bps = 1;  return;
    case 1: *_ms = MOD_QPSK;    *_bps = 2;  return;
    case 2: *_ms = MOD_PSK;     *_bps = 3;  return;
    case 3: *_ms = MOD_PSK;     *_bps = 4;  return;
    case 4: *_ms = MOD_QAM;     *_bps = 4;  return;
    case 5: *_ms = MOD_QAM;     *_bps = 5;  return;
    case 6: *_ms = MOD_QAM;     *_bps = 6;  return;
    default:
        fprintf(stderr,"warning: parameter_get_mod_scheme(), invalid configuration (setting to BPSK)\n");
        *_ms = MOD_BPSK;
        *_bps = 1;
    }
}

void parameter_set_fec_scheme(parameter _p, fec_scheme _fs)
{
    if (_fs == FEC_REP5) _fs = FEC_NONE;
    parameter_set_discrete_value(_p, (unsigned int)_fs);
}


void parameter_get_fec_scheme(parameter _p, fec_scheme * _fs)
{
    *_fs = (fec_scheme) parameter_get_discrete_value(_p);
    if (*_fs == FEC_REP5) *_fs = FEC_NONE;

}

void mutate_parameters(modulation_scheme * _ms,
                       unsigned int * _bps,
                       fec_scheme * _fec0,
                       fec_scheme * _fec1,
                       unsigned int * _payload_len,
                       float * _tx_gain_dB)
{
    // modulation scheme
    if ( (rand()%8)==0 ) {
        unsigned int v = rand() % NUM_MOD_SCHEMES;
        switch (v) {
        case 0: *_ms = MOD_BPSK;    *_bps = 1; break;
        case 1: *_ms = MOD_QPSK;    *_bps = 2; break;
        case 2: *_ms = MOD_PSK;     *_bps = 3; break;
        case 3: *_ms = MOD_PSK;     *_bps = 4; break;
        case 4: *_ms = MOD_QAM;     *_bps = 4; break;
        case 5: *_ms = MOD_QAM;     *_bps = 5; break;
        case 6: *_ms = MOD_QAM;     *_bps = 6; break;
        default:
            fprintf(stderr,"error: mutate_parameters(), invalid mod scheme\n");
            exit(1);
        }
    }

    // fec scheme(s)
    if ( (rand()%8)==0 ) {
        do {
            *_fec0 = (fec_scheme) (rand() % LIQUID_NUM_FEC_SCHEMES);
        } while (*_fec0 == FEC_UNKNOWN || *_fec0 == FEC_REP5);
    }

    // payload length
    int n = (int)(*_payload_len) + (int)(rand()%51) - 25;
    if (n <= 0)         *_payload_len = 1;
    else if (n > 1023)  *_payload_len = 1023;
    else                *_payload_len = (unsigned int)n;
    if ( (rand()%10)==0 )
        *_payload_len = rand() % 1024;

    // transmit gain
    *_tx_gain_dB += randnf() * 0.4f;
    if (*_tx_gain_dB >   0.0f) *_tx_gain_dB =   0.0f;
    if (*_tx_gain_dB < -25.0f) *_tx_gain_dB = -25.0f;
}

