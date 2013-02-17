/*
 * Copyright (c) 2013 Joseph Gaeddert
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
 
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <complex>
#include <getopt.h>
#include <pthread.h>
#include <liquid/liquid.h>

#include <uhd/usrp/multi_usrp.hpp>

#include "timer.h"

void usage() {
    printf("fullduplex_txrx [OPTION]\n");
    printf("transmit OFDM packets back and forth\n");
    printf("\n");
    printf("  h     : usage/help\n");
    printf("  v/q   : verbose/quiet\n");
    printf("  f     : tx frequency [Hz],        default:  462 MHz\n");
    printf("  o     : rx frequency offset [Hz], default:  +40 MHz\n");
    printf("  R     : reverse tx/rx frequencies,default: false\n");
    printf("  b     : bandwidth [Hz],           default:  400 kHz\n");
    printf("  g     : software tx gain [dB],    default:  -12 dB\n");
    printf("  G     : uhd tx gain [dB],         default:   40 dB\n");
    printf("  N     : number of frames,         default: 1000\n");
    printf("  M     : number of subcarriers,    default:   64\n");
    printf("  C     : cyclic prefix length,     default:    8\n");
    printf("  T     : taper length,             default:    4\n");
    printf("  P     : payload length,           default:  800 bytes\n");
    printf("  m     : modulation scheme,        defulat: qpsk\n");
    liquid_print_modulation_schemes();
    printf("  c     : coding scheme (inner),    default: h128\n");
    printf("  k     : coding scheme (outer),    default: none\n");
    liquid_print_fec_schemes();
}

// threads
void * tx_worker(void * _userdata);
void * rx_worker(void * _userdata);

// command-line options
bool verbose = true;

double frequency        = 462.0e6;  // tx frequency
double offset           =   40e6f;  // rx frequency offset
int reverse_txrx        = 0;        // reverse tx/rx frequencies
double bandwidth        = 800e3f;   // bandwidth
unsigned int num_frames = 1000;     // number of frames to transmit
double txgain_dB        = -12.0f;   // software tx gain [dB]
double uhd_txgain       = 40.0;     // uhd (hardware) tx gain

// receiver framing properties
unsigned int M          = 32;       // number of subcarriers
unsigned int cp_len     = 6;        // cyclic prefix length
unsigned int taper_len  = 4;        // taper length

// transmitter framing properties
modulation_scheme ms    = LIQUID_MODEM_QPSK;    // modulation scheme
unsigned int payload_len= 800;                  // original data message length
crc_scheme check        = LIQUID_CRC_32;        // data validity check
fec_scheme fec0         = LIQUID_FEC_HAMMING128;// fec (inner)
fec_scheme fec1         = LIQUID_FEC_NONE;      // fec (outer)
    
// receiver data counters
unsigned int num_frames_detected;
unsigned int num_valid_headers_received;
unsigned int num_valid_packets_received;
unsigned int num_valid_bytes_received;

// receiver callback function
int callback(unsigned char *  _header,
             int              _header_valid,
             unsigned char *  _payload,
             unsigned int     _payload_len,
             int              _payload_valid,
             framesyncstats_s _stats,
             void *           _userdata);

int main (int argc, char **argv)
{
    //
    int d;
    while ((d = getopt(argc,argv,"hvqf:o:Rb:g:G:N:M:C:T:P:m:c:k:")) != EOF) {
        switch (d) {
        case 'h':   usage();                        return 0;
        case 'v':   verbose     = true;             break;
        case 'q':   verbose     = false;            break;
        case 'f':   frequency   = atof(optarg);     break;
        case 'o':   offset      = atof(optarg);     break;
        case 'R':   reverse_txrx= 1;                break;
        case 'b':   bandwidth   = atof(optarg);     break;
        case 'g':   txgain_dB   = atof(optarg);     break;
        case 'G':   uhd_txgain  = atof(optarg);     break;
        case 'N':   num_frames  = atoi(optarg);     break;
        case 'M':   M           = atoi(optarg);     break;
        case 'C':   cp_len      = atoi(optarg);     break;
        case 'T':   taper_len   = atoi(optarg);     break;
        case 'P':   payload_len = atoi(optarg);     break;
        case 'm':   ms          = liquid_getopt_str2mod(optarg);    break;
        case 'c':   fec0        = liquid_getopt_str2fec(optarg);    break;
        case 'k':   fec1        = liquid_getopt_str2fec(optarg);    break;
        default:    usage();                        return 0;
        }
    }

    if (cp_len == 0 || cp_len > M) {
        fprintf(stderr,"error: %s, cyclic prefix must be in (0,M]\n", argv[0]);
        exit(1);
    } else if (ms == LIQUID_MODEM_UNKNOWN) {
        fprintf(stderr,"error: %s, unknown/unsupported mod. scheme\n", argv[0]);
        exit(-1);
    } else if (fec0 == LIQUID_FEC_UNKNOWN) {
        fprintf(stderr,"error: %s, unknown/unsupported inner fec scheme\n", argv[0]);
        exit(-1);
    } else if (fec1 == LIQUID_FEC_UNKNOWN) {
        fprintf(stderr,"error: %s, unknown/unsupported outer fec scheme\n", argv[0]);
        exit(-1);
    }

    // initialize threads
    pthread_t tx_process;   // transmit thread
    pthread_t rx_process;   // receive thread
    pthread_attr_t thread_attr; // threading attributes
    void * status;

    // set thread attributes
    pthread_attr_init(&thread_attr);
    pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_JOINABLE);

    // create threads
    pthread_create(&tx_process, &thread_attr, tx_worker, NULL);
    pthread_create(&rx_process, &thread_attr, rx_worker, NULL);

    // attributes object no longer needed
    pthread_attr_destroy(&thread_attr);

    // join threads
    pthread_join(tx_process, &status);
    pthread_join(rx_process, &status);

    //
    printf("main process complete.\n");

    return 0;
}

// threads
void * tx_worker(void * _args)
{
    // options
    double tx_frequency = reverse_txrx ? frequency + offset : frequency;

    uhd::device_addr_t dev_addr;
    //dev_addr["addr0"] = "192.168.10.2";
    //dev_addr["addr1"] = "192.168.10.3";
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);

    // try to set tx rate (oversampled to compensate for CIC filter)
    usrp->set_tx_rate(3.0f * bandwidth);

    // get actual tx rate
    double usrp_tx_rate = usrp->get_tx_rate();

    // compute arbitrary resampling rate (make up the difference in software)
    double tx_resamp_rate = usrp_tx_rate / bandwidth;

    usrp->set_tx_freq(tx_frequency);
    usrp->set_tx_gain(uhd_txgain);
    usrp->set_tx_antenna("TX/RX");

    printf("tx frequency    :   %10.4f [MHz]\n", tx_frequency*1e-6f);
    printf("bandwidth       :   %10.4f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity       :   %s\n", (verbose?"enabled":"disabled"));
    printf("usrp sample rate:   %10.4f kHz = %10.4f kHz * %8.6f\n",
            usrp_tx_rate * 1e-3f,
            bandwidth    * 1e-3f,
            tx_resamp_rate);

    // set the IF filter bandwidth
    //usrp->set_tx_bandwidth(2.0f*bandwidth);

    // add arbitrary resampling component
    msresamp_crcf resamp = msresamp_crcf_create(tx_resamp_rate, 60.0f);

    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/20.0f);

    // data arrays
    unsigned char header[8];
    unsigned char payload[payload_len];
    
    // create frame generator (default subcarrier allocation)
    ofdmflexframegenprops_s fgprops;
    ofdmflexframegenprops_init_default(&fgprops);
    fgprops.check           = check;
    fgprops.fec0            = fec0;
    fgprops.fec1            = fec1;
    fgprops.mod_scheme      = ms;
    ofdmflexframegen fg = ofdmflexframegen_create(M, cp_len, taper_len, NULL, &fgprops);
    ofdmflexframegen_print(fg);

    // allocate array to hold each OFDM symbol
    unsigned int symbol_len = M + cp_len;   // ofdm symbol length (samples)
    std::complex<float> ofdm_symbol[symbol_len]; // output time series of each symbol

    // create buffer for arbitrary resamper output
    std::complex<float> buffer_resamp[(int)(2*tx_resamp_rate) + 64];

    // vector buffer to send data to USRP
    std::vector<std::complex<float> > usrp_buffer(256);
    unsigned int usrp_sample_counter = 0;

    // set up the metadta flags
    uhd::tx_metadata_t md;
    md.start_of_burst = false;  // never SOB when continuous
    md.end_of_burst   = false;  // 
    md.has_time_spec  = false;  // set to false to send immediately
    //md.time_spec = uhd::time_spec_t(0.1);

    unsigned int j;
    unsigned int pid;
    for (pid=0; pid<num_frames; pid++) {
        // reset frame generator (resets pilot generator, etc.)
        ofdmflexframegen_reset(fg);

        if (verbose)
            printf("tx packet id: %6u\n", pid);
        
        // write header (first two bytes packet ID, remaining are random)
        header[0] = (pid >> 8) & 0xff;
        header[1] = (pid     ) & 0xff;
        for (j=2; j<8; j++)
            header[j] = rand() & 0xff;

        // initialize payload
        for (j=0; j<payload_len; j++)
            payload[j] = rand() & 0xff;

        // assemble frame
        ofdmflexframegen_assemble(fg, header, payload, payload_len);

        // generate a single OFDM frame
        int last_symbol=0;
        unsigned int zero_pad=1;
        while (!last_symbol || zero_pad > 0) {

            // generate OFDM symbol
            if (!last_symbol) {
                // generate symbol
                last_symbol = ofdmflexframegen_writesymbol(fg, ofdm_symbol);
            } else {
                zero_pad--;
                for (j=0; j<symbol_len; j++)
                    ofdm_symbol[j] = 0.0f;
            }

            // resample OFDM symbol and push resulting samples to USRP
            for (j=0; j<symbol_len; j++) {
                // resample OFDM symbol one sample at a time
                unsigned int nw;    // number of samples output from resampler
                msresamp_crcf_execute(resamp, &ofdm_symbol[j], 1, buffer_resamp, &nw);

                // for each output sample, stuff into USRP buffer
                unsigned int n;
                for (n=0; n<nw; n++) {
                    // append to USRP buffer, scaling by software
                    usrp_buffer[usrp_sample_counter++] = g*buffer_resamp[n];

                    // once USRP buffer is full, reset counter and send to device
                    if (usrp_sample_counter==256) {
                        // reset counter
                        usrp_sample_counter=0;

                        // send the result to the USRP
                        usrp->get_device()->send(
                            &usrp_buffer.front(), usrp_buffer.size(), md,
                            uhd::io_type_t::COMPLEX_FLOAT32,
                            uhd::device::SEND_MODE_FULL_BUFF
                        );
                    }
                }
            }

        } // while loop


    } // packet loop
 
    // send a mini EOB packet
    md.start_of_burst = false;
    md.end_of_burst   = true;
    usrp->get_device()->send("", 0, md,
        uhd::io_type_t::COMPLEX_FLOAT32,
        uhd::device::SEND_MODE_FULL_BUFF
    );

    // sleep for a small amount of time to allow USRP buffers
    // to flush
    usleep(100000);

    // delete allocated objects
    ofdmflexframegen_destroy(fg);
    msresamp_crcf_destroy(resamp);
    
    // finished
    printf("tx worker finished\n");

    // return
    pthread_exit(0);
}

void * rx_worker(void * _args)
{
    // options
    double uhd_rxgain = 20.0f;
    double num_seconds = 600.0f;
    int debug_enabled = 0;
    double rx_frequency = reverse_txrx ? frequency : frequency + offset;

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);

    stream_cmd.stream_now = true;

    uhd::device_addr_t dev_addr;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);

    // try to set rx rate (oversampled to compensate for CIC filter)
    usrp->set_rx_rate(3.0f * bandwidth);

    // get actual rx rate
    double usrp_rx_rate = usrp->get_rx_rate();

    // compute arbitrary resampling rate (make up the difference in software)
    double rx_resamp_rate = bandwidth / usrp_rx_rate;

    usrp->set_rx_freq(rx_frequency);
    usrp->set_rx_gain(uhd_rxgain);
    usrp->set_rx_antenna("RX2");

    printf("rx frequency    :   %10.4f [MHz]\n", rx_frequency*1e-6f);
    printf("bandwidth       :   %10.4f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity       :   %s\n", (verbose?"enabled":"disabled"));
    printf("usrp sample rate:   %10.4f kHz = %10.4f kHz * %8.6f\n",
            usrp_rx_rate * 1e-3f,
            bandwidth    * 1e-3f,
            1.0f / rx_resamp_rate);

    // set run time appropriately
    if (num_seconds < 0) {
        num_seconds = 1e32; // set to really, really large number
        printf("run time        :   (forever)\n");
    } else {
        printf("run time        :   %f seconds\n", num_seconds);
    }

    // add arbitrary resampling component
    msresamp_crcf resamp = msresamp_crcf_create(rx_resamp_rate, 60.0f);

    unsigned int block_len = 64;
    assert( (block_len % 2) == 0);  // ensure block length is even

    //allocate recv buffer and metatdata
    uhd::rx_metadata_t md;
    const size_t max_samps_per_packet = usrp->get_device()->get_max_recv_samps_per_packet();
    std::vector<std::complex<float> > buff(max_samps_per_packet);

    // create frame synchronizer (default subcarrier allocation)
    ofdmflexframesync fs = ofdmflexframesync_create(M,cp_len,taper_len,NULL,callback,(void*)&bandwidth);
    if (debug_enabled)
        ofdmflexframesync_debug_enable(fs);
    ofdmflexframesync_print(fs);

    // start data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    printf("usrp data transfer started\n");
 
    // create buffer for arbitrary resamper output
    std::complex<float> buffer_resamp[(int)(2.0f/rx_resamp_rate) + 64];
 
    // reset counters
    num_frames_detected=0;
    num_valid_headers_received=0;
    num_valid_packets_received=0;
    num_valid_bytes_received=0;

    // run conditions
    int continue_running = 1;
    timer t0 = timer_create();
    timer_tic(t0);

    while (continue_running) {
        // grab data from device
        size_t num_rx_samps = usrp->get_device()->recv(
            &buff.front(), buff.size(), md,
            uhd::io_type_t::COMPLEX_FLOAT32,
            uhd::device::RECV_MODE_ONE_PACKET
        );

        // 'handle' the error codes
        switch(md.error_code){
        case uhd::rx_metadata_t::ERROR_CODE_NONE:
        case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
            break;

        default:
            std::cerr << "Error code: " << md.error_code << std::endl;
            std::cerr << "Unexpected error on recv, exit test..." << std::endl;
            //return 1;
        }

        // push data through arbitrary resampler and give to frame synchronizer
        // TODO : apply bandwidth-dependent gain
        unsigned int j;
        for (j=0; j<num_rx_samps; j++) {
            // grab sample from usrp buffer
            std::complex<float> usrp_sample = buff[j];

            // push through resampler (one at a time)
            unsigned int nw;
            msresamp_crcf_execute(resamp, &usrp_sample, 1, buffer_resamp, &nw);

            // push resulting samples through synchronizer
            ofdmflexframesync_execute(fs, buffer_resamp, nw);
        }

        // check runtime
        if (timer_toc(t0) >= num_seconds)
            continue_running = 0;
    }
 
    // compute actual run-time
    float runtime = timer_toc(t0);

    // stop data transfer
    usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    printf("\n");
    printf("usrp data transfer complete\n");
 
    // print results
    float data_rate = num_valid_bytes_received * 8.0f / num_seconds;
    float percent_headers_valid = (num_frames_detected == 0) ?
                          0.0f :
                          100.0f * (float)num_valid_headers_received / (float)num_frames_detected;
    float percent_packets_valid = (num_frames_detected == 0) ?
                          0.0f :
                          100.0f * (float)num_valid_packets_received / (float)num_frames_detected;
    printf("    frames detected     : %6u\n", num_frames_detected);
    printf("    valid headers       : %6u (%6.2f%%)\n", num_valid_headers_received,percent_headers_valid);
    printf("    valid packets       : %6u (%6.2f%%)\n", num_valid_packets_received,percent_packets_valid);
    printf("    bytes received      : %6u\n", num_valid_bytes_received);
    printf("    run time            : %f s\n", runtime);
    printf("    data rate           : %8.4f kbps\n", data_rate*1e-3f);

    // export debugging file
    if (debug_enabled)
        ofdmflexframesync_debug_print(fs, "ofdmflexframesync_debug.m");

    // destroy objects
    msresamp_crcf_destroy(resamp);
    ofdmflexframesync_destroy(fs);
    timer_destroy(t0);

    // finished
    printf("rx worker finished\n");

    // return
    pthread_exit(0);
}

// callback function
int callback(unsigned char *  _header,
             int              _header_valid,
             unsigned char *  _payload,
             unsigned int     _payload_len,
             int              _payload_valid,
             framesyncstats_s _stats,
             void *           _userdata)
{
    if (verbose) {
        // compute true carrier offset
        double samplerate = *((double*)_userdata);
        float cfo = _stats.cfo * samplerate / (2*M_PI);
        printf("***** rssi=%7.2fdB evm=%7.2fdB, cfo=%7.3f kHz, ", _stats.rssi, _stats.evm, cfo*1e-3f);

        if (_header_valid) {
            unsigned int packet_id = (_header[0] << 8 | _header[1]);
            printf("rx packet id: %6u", packet_id);
            if (_payload_valid) printf("\n");
            else                printf(" PAYLOAD INVALID\n");
        } else {
            printf("HEADER INVALID\n");
        }
    } else {
    }

    // update global counters
    num_frames_detected++;

    if (_header_valid)
        num_valid_headers_received++;

    if (_payload_valid) {
        num_valid_packets_received++;
        num_valid_bytes_received += _payload_len;
    }

    return 0;
}

