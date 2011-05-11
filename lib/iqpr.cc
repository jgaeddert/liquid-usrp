/*
 * Copyright (c) 2010 Joseph Gaeddert
 * Copyright (c) 2010 Virginia Polytechnic Institute & State University
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
// iqpr.cc
//
// iqpr: l(iq)uid (p)acket (r)adio
//

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <complex>

#include <liquid/liquid.h>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/usrp/single_usrp.hpp>

#include "iqpr.h"

// iqpr data structure
struct iqpr_s {
    // UHD interface to hardware
    uhd::usrp::single_usrp::sptr usrp;

    // receiver
    std::vector<std::complex<float> > rx_buffer;    // rx data buffer
    unsigned int rx_vector_index;                   // index of rx buffer vector
    unsigned int rx_vector_length;                  // length of rx buffer vector
    resamp2_crcf rx_decim;          // half-band decimator
    resamp_crcf rx_resamp;          // arbitrary resampler
    framesyncprops_s fsprops;       // frame synchronizer properties
    flexframesync fs;               // frame synchronizer
    int rx_packet_found;            // receiver packet found flag
    unsigned char * rx_data;        // received payload data
    unsigned int rx_data_len;       // received payload data length
    framesyncstats_s rx_stats;      // frame synchronizer stats

    // transmitter
    unsigned char * tx_data;        // transmitted payload data
    unsigned int tx_data_len;       // transmitted payload data length
    flexframegenprops_s fgprops;    // frame generator properties
    flexframegen fg;                // frame generator
    float tx_gain;                  // transmit gain
    interp_crcf tx_interp;          // matched-filter interpolator
    resamp_crcf tx_resamp;          // arbitrary resampler
    //std::complex<float> * frame;    // frame buffer
    std::complex<float> data_tx[32];                // tx data buffer (array)
    std::complex<float> data_tx_interp[128];        // tx interpolated
    std::complex<float> data_tx_resamp[256];        // tx resampled
    std::vector<std::complex<float> > tx_buffer;    // tx data buffer

    // debugging
    int verbose;

    // 
    // higher-level functionality
    //
    unsigned int node_id;           // node identifier
};


// create iqpr object
iqpr iqpr_create()
{
    // allocate memory for main object
    iqpr q = (iqpr) malloc(sizeof(struct iqpr_s));

    // TODO : create USRP object
    uhd::device_addr_t dev_addr;
    // TODO : set up address as necessary
    q->usrp = uhd::usrp::single_usrp::make(dev_addr);

    // 
    // receiver objects
    //
    const size_t max_samps_per_packet = q->usrp->get_device()->get_max_recv_samps_per_packet();
    q->rx_buffer.resize(max_samps_per_packet);
    q->rx_vector_index  = 0;
    q->rx_vector_length = 0;
    q->rx_resamp = resamp_crcf_create(1.0, 7, 0.4, 60.0, 64);
    q->rx_decim = resamp2_crcf_create(7, 0.0, 40.0);

    // create frame synchronizer
    framesyncprops_init_default(&q->fsprops);
    q->fsprops.squelch_threshold = -37.0f;
    q->fsprops.squelch_enabled = 1;
    q->fs = flexframesync_create(&q->fsprops, iqpr_callback, (void*)q);

    // allocate memory for received data
    q->rx_data_len = 1024;
    q->rx_data = (unsigned char*) malloc(q->rx_data_len*sizeof(unsigned char));


    // 
    // transmitter objects
    //

    // create frame generator
    flexframegenprops_init_default(&q->fgprops);
    q->fgprops.rampup_len   = 16;
    q->fgprops.phasing_len  = 64;
    q->fgprops.payload_len  = 0;
    q->fgprops.check        = LIQUID_CRC_NONE;
    q->fgprops.fec0         = LIQUID_FEC_NONE;
    q->fgprops.fec1         = LIQUID_FEC_NONE;
    q->fgprops.mod_scheme   = LIQUID_MODEM_QPSK;
    q->fgprops.mod_bps      = 2;
    q->fgprops.rampdn_len   = 16;
    q->fg = flexframegen_create(&q->fgprops);
    flexframegen_print(q->fg);

    // create interpolator
    unsigned int k=4;
    unsigned int m=3;
    float beta=0.7f;
    q->tx_interp = interp_crcf_create_rnyquist(LIQUID_RNYQUIST_RRC,k,m,beta,0);

    q->tx_resamp = resamp_crcf_create(1.0, 7, 0.4, 60.0, 64);

    //q->tx_buffer.resize(1);

    // set transmit gain
    q->tx_gain = 1.0f;

    // debugging
    q->verbose = 0;

    return q;
}

void iqpr_destroy(iqpr _q)
{
    // 
    // receiver objects
    //
    resamp_crcf_destroy(_q->rx_resamp);
    resamp2_crcf_destroy(_q->rx_decim);
    flexframesync_destroy(_q->fs);
    free(_q->rx_data);

    // 
    // transmitter objects
    //
    flexframegen_destroy(_q->fg);
    interp_crcf_destroy(_q->tx_interp);
    resamp_crcf_destroy(_q->tx_resamp);

    // free main object memory
    free(_q);
}

void iqpr_print(iqpr _q)
{
    printf("iqpr:\n");
}

// set verbosity on
void iqpr_set_verbose(iqpr _q)
{
    _q->verbose = 1;
}

// set verbosity off
void iqpr_unset_verbose(iqpr _q)
{
    _q->verbose = 0;
}

// set transmit/receive hardware gain
void iqpr_set_tx_gain(iqpr _q, float _tx_gain)
{
    _q->usrp->set_tx_gain(_tx_gain);
}

// set transmit/receive hardware gain
void iqpr_set_rx_gain(iqpr _q, float _rx_gain)
{
    _q->usrp->set_rx_gain(_rx_gain);
}

#if 0
// set transmit/receive software gain
void iqpr_set_tx_power(iqpr _q, float _tx_power);
void iqpr_set_rx_power(iqpr _q, float _rx_power);
#endif

// set transmit/receive sample rate
void iqpr_set_tx_rate(iqpr _q, float _tx_rate)
{
    unsigned long int DAC_RATE = 64e6;

    // over-sampling by a factor of 4
    _tx_rate *= 4.0;

    unsigned int interp_rate = (unsigned int)(DAC_RATE / _tx_rate);
    // ensure multiple of 4
    interp_rate = (interp_rate >> 2) << 2;
    interp_rate += 4; // ensure tx_resamp_rate <= 1.0

    // compute usrp sampling rate
    double usrp_tx_rate = DAC_RATE / (double)interp_rate;

    // compute arbitrary resampling rate
    double tx_resamp_rate = usrp_tx_rate / _tx_rate;
    printf("sample rate :   %12.8f kHz = %12.8f / %8.6f (interp %u)\n",
            _tx_rate * 1e-3f,
            usrp_tx_rate * 1e-3f,
            tx_resamp_rate,
            interp_rate);

    // set hardware sampling rate
    _q->usrp->set_tx_rate(DAC_RATE / interp_rate);

    // set software resampling rate
    resamp_crcf_setrate(_q->tx_resamp, tx_resamp_rate);
}

// set transmit/receive sample rate
void iqpr_set_rx_rate(iqpr _q, float _rx_rate)
{
    unsigned long int ADC_RATE = 64e6;

    // over-sampling by a factor of 4
    _rx_rate *= 4.0;

    unsigned int decim_rate = (unsigned int)(ADC_RATE / _rx_rate);
    // ensure multiple of 2
    decim_rate = (decim_rate >> 1) << 1;

    // compute usrp sampling rate
    double usrp_rx_rate = ADC_RATE / decim_rate;

    // compute arbitrary resampling rate
    double rx_resamp_rate = _rx_rate / usrp_rx_rate;
    printf("sample rate :   %12.8f kHz = %12.8f * %8.6f (decim %u)\n",
            _rx_rate * 1e-3f,
            usrp_rx_rate * 1e-3f,
            rx_resamp_rate,
            decim_rate);

    // set hardware sampling rate
    _q->usrp->set_rx_rate(ADC_RATE / decim_rate);

    // set software resampling rate
    resamp_crcf_setrate(_q->rx_resamp, rx_resamp_rate);
}

// set transmit/receive frequency
void iqpr_set_tx_freq(iqpr _q, float _tx_freq)
{
    _q->usrp->set_tx_freq(_tx_freq);
}

// set transmit/receive frequency
void iqpr_set_rx_freq(iqpr _q, float _rx_freq)
{
    _q->usrp->set_rx_freq(_rx_freq);
}

// set flexframesync properties (receiver)
void iqpr_rxconfig(iqpr _q,
                   framesyncprops_s * _fsprops)
{
    flexframesync_setprops(_q->fs, _fsprops);
}


// start data transfer
void iqpr_rx_start(iqpr _q)
{
    _q->usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
} 


// stop data transfer
void iqpr_rx_stop(iqpr _q)
{
    _q->usrp->issue_stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
} 


// 
// low-level functionality
//

// transmit packet (generic)
//  _q              :   iqpr object
//  _header         :   header data [14 bytes]
//  _payload        :   payload data
//  _payload_len    :   number of bytes in payload
//  _fgprops        :   frame generator properties (internal 'payload_len' ignored)
void iqpr_txpacket(iqpr _q,
                   unsigned char * _header,
                   unsigned char * _payload,
                   unsigned int _payload_len,
                   flexframegenprops_s * _fgprops)
{
    //unsigned int i;

    // configure frame generator
    if (_fgprops != NULL)
        memmove(&_q->fgprops, _fgprops, sizeof(flexframegenprops_s));
    _q->fgprops.payload_len = _payload_len;
    flexframegen_setprops(_q->fg, &_q->fgprops);

    unsigned int frame_len = flexframegen_getframelen(_q->fg);
    //printf("frame length : %u\n", frame_len);

    // set up the metadta flags
    uhd::tx_metadata_t md;
    md.start_of_burst = false;  // never SOB when continuous
    md.end_of_burst   = false;  // 
    md.has_time_spec  = false;  // set to false to send immediately

    std::complex<float> frame[frame_len];       // frame

    //printf("[tx] sending packet %u...\n", _pid);

    // generate frame
    flexframegen_execute(_q->fg, _header, _payload, frame);

    std::complex<float> buffer_interp[4*frame_len];  // matched-filter interpolator (interp by 4)
    std::complex<float> buffer_resamp[6*frame_len]; // resampler
    std::vector<std::complex<float> > buff(6*frame_len);

    unsigned int j;
    float g = 0.3f;
    // interpolate using matched filter
    for (j=0; j<frame_len; j++)
        interp_crcf_execute(_q->tx_interp, g*frame[j], &buffer_interp[4*j]);
        
    // run resampler
    unsigned int n=0;
    unsigned int nw;
    for (j=0; j<4*frame_len; j++) {
        resamp_crcf_execute(_q->tx_resamp, buffer_interp[j], &buffer_resamp[n], &nw);
        n += nw;
    }

    //printf(" n = %6u (frame_len : %6u)\n", n, frame_len);
    buff.resize(n);
    for (j=0; j<n; j++) {
        buff[j] = buffer_resamp[j];
    }

    //send the entire contents of the buffer
    _q->usrp->get_device()->send(
        &buff.front(), buff.size(), md,
        uhd::io_type_t::COMPLEX_FLOAT32,
        uhd::device::SEND_MODE_FULL_BUFF
    );

#if 0
    // send a mini EOB packet
    md.start_of_burst = false;
    md.end_of_burst   = true;
    _q->usrp->get_device()->send("", 0, md,
        uhd::io_type_t::COMPLEX_FLOAT32,
        uhd::device::SEND_MODE_FULL_BUFF
    );
#endif
}

// receive data packet with timeout, returning 1 if found, 0 if not
//  _q              :   iqpr object
//  _timespec       :   time specifier
//  _header         :   received header
//  _header_valid   :   header valid?
//  _payload        :   output payload data
//  _payload_len    :   number of bytes in payload
//  _payload_valid  :   payload valid?
//  _stats          :   received frame statistics
int iqpr_rxpacket(iqpr _q,
                  unsigned int _timespec,
                  unsigned char ** _header,
                  int           *  _header_valid,
                  unsigned char ** _payload,
                  unsigned int  *  _payload_len,
                  int           *  _payload_valid,
                  framesyncstats_s * _stats)
{
    unsigned long int total_samples = 100000;
    // TODO : start 'timer'
    unsigned long int num_accumulated_samples = 0;

    uhd::rx_metadata_t md;

    // buffers
    std::complex<float> rx_buffer_decim[2];
    std::complex<float> rx_decim_out;
    std::complex<float> rx_buffer_resamp[4];

    // read samples from buffer, run through frame synchronizer
    while ( num_accumulated_samples < total_samples) {

        // check if we have read entire contents of buffer
        if (_q->rx_vector_index == _q->rx_vector_length) {
            // reset vector index
            _q->rx_vector_index = 0;

            // grab data from port
            _q->rx_vector_length = _q->usrp->get_device()->recv(
                &_q->rx_buffer.front(),
                _q->rx_buffer.size(),
                md,
                uhd::io_type_t::COMPLEX_FLOAT32,
                uhd::device::RECV_MODE_ONE_PACKET
            );

            //handle the error codes
            switch(md.error_code){
            case uhd::rx_metadata_t::ERROR_CODE_NONE:
            case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
                break;
            default:
                std::cerr << "Error code: " << md.error_code << std::endl;
                std::cerr << "Unexpected error on recv, exit test..." << std::endl;
                exit(1);
            }
        }

        // for now copy vector "buff" to array of complex float
        // TODO : apply bandwidth-dependent gain
        unsigned int n=0;
        unsigned int nw=0;
        while (_q->rx_vector_index < _q->rx_vector_length) {
            num_accumulated_samples++;

            // push 2 samples into buffer
            rx_buffer_decim[n++] = _q->rx_buffer[_q->rx_vector_index++];

            if (n==2) {
                // reset counter
                n=0;

                // decimate
                resamp2_crcf_decim_execute(_q->rx_decim, rx_buffer_decim, &rx_decim_out);

                // apply resampler
                resamp_crcf_execute(_q->rx_resamp, rx_decim_out, rx_buffer_resamp, &nw);

                // push through synchronizer
                flexframesync_execute(_q->fs, rx_buffer_resamp, nw);

#if 0
                // check status flag
                if (_q->rx_packet_found) {
                    if (_q->verbose) printf(" received data packet %u!\n", _q->rx_header.pid);

                    // set outputs
                    *_payload = _q->rx_data;
                    *_payload_len = _q->rx_data_len;
                    memmove(_header, &_q->rx_header, sizeof(struct iqprheader_s));
                    memmove(_stats,  &_q->rx_stats,  sizeof(framesyncstats_s));
                    return 1;
                }
#endif
            } // resamp
        } // while
    }

    // packet was never received
    return 0;
}

#if 0
void iqpr_txack(iqpr _q,
                unsigned int _pid)
{
    // configure frame generator
    _q->fgprops.payload_len = 0;
    flexframegen_setprops(_q->fg, &_q->fgprops);

    unsigned int i;

    unsigned int frame_len = flexframegen_getframelen(_q->fg);
    std::complex<float> frame[frame_len];
    std::complex<float> mfbuffer[2*frame_len];

    if (_q->verbose) printf("[tx] sending ack for packet %u...\n", _pid);

    // prepare header
    _q->tx_header.pid = _pid;
    _q->tx_header.packet_type = IQPR_PACKET_TYPE_ACK;
    _q->tx_header.node_src = _q->node_id;
    _q->tx_header.node_dst = 0;

    unsigned char header[8];
    iqprheader_encode(&_q->tx_header, header);

    // configure frame generator
    _q->fgprops.payload_len = 0;
    _q->fgprops.mod_scheme  = LIQUID_MODEM_BPSK;
    _q->fgprops.mod_bps     = 1;
    _q->fgprops.check       = LIQUID_CRC_NONE;
    _q->fgprops.fec0        = LIQUID_FEC_NONE;
    _q->fgprops.fec1        = LIQUID_FEC_NONE;
    flexframegen_setprops(_q->fg, &_q->fgprops);

    // generate frame
    flexframegen_execute(_q->fg, header, NULL, frame);

    // run interpolator
    for (i=0; i<frame_len; i++) {
        interp_crcf_execute(_q->interp, frame[i]*_q->tx_gain, &mfbuffer[2*i]);
    }

    // produce data in buffer
#if 0
    gport_produce(_q->port_tx, (void*)mfbuffer, 2*frame_len);
#endif

    // flush interpolator with zeros
    for (i=0; i<64; i++) {
        interp_crcf_execute(_q->interp, 0, &mfbuffer[2*i]);
    }

#if 0
    gport_produce(_q->port_tx, (void*)mfbuffer, 128);
#endif
}

int iqpr_wait_for_ack(iqpr _q,
                      unsigned int _pid,
                      iqprheader_s * _header,
                      framesyncstats_s * _stats)
{
    // set ack to trigger on a specific packet id
    _q->rx_state = IQPR_RX_WAIT_FOR_ACK;
    _q->rx_packet_pid = _pid;
    _q->rx_packet_found = 0;

    // read samples from buffer, run through frame synchronizer
    unsigned int i;
    for (i=0; i<10; i++) {
        // grab data from port
#if 0
        gport_consume(_q->port_rx, (void*)_q->rx_buffer, 512);
#endif

        // run through frame synchronizer
        flexframesync_execute(_q->fs, _q->rx_buffer, 512);

        // check status flag
        if (_q->rx_packet_found) {
            if (_q->verbose) printf(" received ack for packet %u!\n", _pid);

            memmove(_header, &_q->rx_header, sizeof(struct iqprheader_s));
            memmove(_stats,  &_q->rx_stats,  sizeof(framesyncstats_s));
            return 1;
        }
    }

    // ack was never received
    return 0;
}


// determine if MAC is clear
float iqpr_mac_getrssi(iqpr _q,
                       unsigned int _num_samples)
{
    // validate input
    if (_num_samples < 1) {
        fprintf(stderr,"warning: iqpr_mac_getrssi(), must use at least 1 sample\n");
        _num_samples = 1;
    }

    // grab data from port
#if 0
    gport_consume(_q->port_rx, (void*)_q->rx_buffer, _num_samples);
#endif

    // estimate signal level
    unsigned int i;
    float rssi = 0.0;
    for (i=0; i<_num_samples; i++)
        rssi += abs(_q->rx_buffer[i]) * abs(_q->rx_buffer[i]);

    // TODO : fix rssi computation
    rssi = 10*log10f(sqrt(rssi/_num_samples));

    if (_q->verbose) {
        printf("mac_clear(), rssi : %12.8f dB %c\n", rssi,
            (rssi < _q->fsprops.squelch_threshold) ? ' ' : '*');
    }

    return rssi;
}
#endif


// 
// iqpr internal methods
//

// iqpr internal callback method
int iqpr_callback(unsigned char * _rx_header,
                  int _rx_header_valid,
                  unsigned char * _rx_payload,
                  unsigned int _rx_payload_len,
                  int _rx_payload_valid,
                  framesyncstats_s _stats,
                  void * _userdata)
{
    printf("********* callback invoked\n");
#if 0
    iqpr q = (iqpr) _userdata;

    if (q->verbose) {
        printf("********* callback invoked, ");
        framesyncstats_print(&_stats);
        // ...
    }

    if ( !_rx_header_valid ) {
        if (q->verbose) printf("header crc : FAIL\n");
        return 0;
    }

    // save statistics
    memmove(&q->rx_stats, &_stats, sizeof(framesyncstats_s));

    // decode header
    iqprheader_decode(&q->rx_header, _rx_header);
    if (q->verbose) printf("packet id: %6u", q->rx_header.pid);

    if (q->verbose) {
        switch (q->rx_header.packet_type) {
        case IQPR_PACKET_TYPE_DATA:  printf(" (data)\n"); break;
        case IQPR_PACKET_TYPE_ACK:   printf(" (ack)\n");  break;
        default:
            printf("\n");
            fprintf(stderr,"error: iqpr_callback(), invaid packet type: %u\n", q->rx_header.packet_type);
        }
    }

    if (q->rx_header.packet_type == IQPR_PACKET_TYPE_ACK) {
        // check to see if we were waiting for an ack and
        // check to see if this packet matches the one we are waiting for
        //
        // TODO : check to see if source/destination ids match as well?
        if ( (q->rx_state == IQPR_RX_WAIT_FOR_ACK) && (q->rx_header.pid == q->rx_packet_pid) ) {
            // set status flag
            q->rx_packet_found = 1;
        } else {
            q->rx_packet_found = 0;
        }

        return 0;
    }

    // re-allocate memory arrays if necessary
    if (q->rx_data_len != _rx_payload_len) {
        q->rx_data_len = _rx_payload_len;
        q->rx_data = (unsigned char*) realloc(q->rx_data, q->rx_data_len*sizeof(unsigned char));
    }

    // copy data (regardless of validity)
    memmove(q->rx_data, _rx_payload, _rx_payload_len*sizeof(unsigned char));

    if (!_rx_payload_valid) {
        if (q->verbose) printf("  <<< payload crc fail >>>\n");

        // payload failed check
        return 0;
    }

    // check to see if we were waiting for a data packet
    if (q->rx_state == IQPR_RX_WAIT_FOR_DATA)
        q->rx_packet_found = 1;

#endif
    return 0;
}

#if 0
// encode header (structure > array)
//  _q      :   iqpr heade structure
//  _header :   8-byte header array
void iqprheader_encode(iqprheader_s * _q,
                       unsigned char * _header)
{
    // encode packet identifier
    _header[0] = (_q->pid >> 8) & 0xff;
    _header[1] = (_q->pid     ) & 0xff;

    // encode packet type
    _header[2] = _q->packet_type & 0xff;

    // encode source and destination nodes
    _header[3] = _q->node_src & 0xff;
    _header[4] = _q->node_dst & 0xff;

    // copy remaining user data
    _header[5] = _q->userdata[0];
    _header[6] = _q->userdata[1];
    _header[7] = _q->userdata[2];
}


// decode header (array > structure))
//  _q      :   iqpr heade structure
//  _header :   8-byte header array
void iqprheader_decode(iqprheader_s * _q,
                       unsigned char * _header)
{
    // decode packet identifier
    _q->pid = (_header[0] << 8) | _header[1];

    // decode packet type
    _q->packet_type = _header[2];

    // decode source and destination nodes
    _q->node_src = _header[3];
    _q->node_dst = _header[4];

    // copy remaining user data
    _q->userdata[0] = _header[5];
    _q->userdata[1] = _header[6];
    _q->userdata[2] = _header[7];
}
#endif

