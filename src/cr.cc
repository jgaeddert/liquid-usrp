//
// Liquid cognitive radio prototype
//
 
#include <math.h>
#include <iostream>
#include <complex>
#include <pthread.h>
#include <time.h>
#include <sys/time.h> // gettimeofday
#include <getopt.h>
#include <liquid/liquid.h>

#include "usrp_standard.h"
#include "usrp_prims.h"
#include "usrp_dbid.h"
#include "usrp_bytesex.h"
#include "flex.h"
#include "usrp_rx_gain_correction.h"

#define VERBOSE             0

#define USRP_CHANNEL        0

#define TX_RATE_MIN         (62.5e3f)
#define TX_RATE_MAX         (2000e3f)
#define RX_RATE_MIN         (62.5e3f)
#define RX_RATE_MAX         (2000e3f)
 
#define OPMODE_MASTER       0
#define OPMODE_SLAVE        1

#define PACKET_TYPE_DATA    0
#define PACKET_TYPE_ACK     1
#define PACKET_TYPE_CONTROL 2

typedef struct pm_header {
    unsigned int src0;  // 0,1
    unsigned int src1;  // 2,3
    unsigned int dst0;  // 4,5
    unsigned int dst1;  // 6,7

    unsigned int pid;   // 8,9
    unsigned int type;  // 10

    // control information
    bool do_set_control;        // 14 enable control change
    unsigned int ctrl_channel;  // 15 move to this channel next packet
    unsigned int ctrl_bandwidth;// 16 set signal bandwidth
    unsigned int ctrl_txgain;   // 17 set transmit gain
    unsigned int ctrl_bch0;     // 18 set primary backup channel
};

typedef struct crdata {
    // fixed
    unsigned short node_id; // identifier for this node (randomly generated)
    int mode;

    // cognitive radio parameters
    float fc;                   // carrier frequency
    float fs_tx;                // tx symbol rate (32e6/interp)
    float fs_rx;                // rx symbol rate (16e6/decim)
    unsigned short tx_gain;     // from 0 to 20,000
    unsigned int ack_timeout;   // time to wait for acknowledgement (ms)
    unsigned int ce_sleep;      // time for ce to sleep (ms)

    unsigned int channel_id;
    unsigned int bandwidth_id;
    unsigned int txgain_id;

    unsigned int num_rx_packets;
    unsigned int num_valid_rx_packets;
    unsigned int num_ack_timeouts;

    // front end objects
    usrp_standard_tx * utx;
    int tx_db_id;
    db_base * txdb;
    usrp_standard_rx * urx;
    int rx_db_id;
    db_base * rxdb;

    //
    unsigned int tx_interp;
    unsigned int rx_decim;

    // threading
    pthread_cond_t  tx_data_ready;
    pthread_cond_t  rx_data_ready;
    pthread_cond_t  control_ready;
    pthread_mutex_t tx_data_mutex;
    pthread_mutex_t rx_data_mutex;
    pthread_mutex_t control_mutex;
    bool radio_active;
    bool ce_waiting;

    // data buffers
    unsigned char tx_header[24];
    unsigned char tx_payload[64];
    bool packet_received;
    pm_header tx_pm_header;

    unsigned char rx_header[24];
    unsigned char rx_payload[64];
    bool rx_header_valid;
    bool rx_payload_valid;
    pm_header rx_pm_header;

    // number of dropped packets before taking action
    unsigned int pm_attempt_timeout;
};

void * tx_process(void*);   // transmitter
void * rx_process(void*);   // receiver
void * pm_process(void*);   // packet manager
void * ce_process(void*);   // cognitive engine

void pm_send_data_packet(crdata * p, unsigned int pid);
void pm_send_ack_packet(crdata * p, unsigned int pid);
bool pm_wait_for_ack_packet(crdata * p, unsigned int pid);
bool pm_wait_for_data_packet(crdata * p, unsigned int pid);

void pm_assemble_header(pm_header _h, unsigned char * _header);
void pm_disassemble_header(unsigned char * _header, pm_header * _h);

float pm_get_channel_frequency(unsigned int _channel)
{
    return 1e6f*(450.0f + _channel*0.1f);
}

float pm_get_bandwidth(unsigned int _bandwidth_id)
{
    return 1e3f*( 3.0f*(_bandwidth_id) + 65.0f);
}

float pm_get_txgain(unsigned int _txgain_id)
{
    return 60.0f*((float)_txgain_id) + 10.0f;
}

void cr_set_tx_frequency(crdata * _p, float _tx_frequency);
void cr_set_rx_frequency(crdata * _p, float _rx_frequency);

void cr_set_tx_symbol_rate(crdata * _p, float _tx_rate);
void cr_set_rx_symbol_rate(crdata * _p, float _rx_rate);

void cr_set_tx_gain(crdata * _p, float _tx_gain);

void cr_set_parameters_by_id(crdata * _p, unsigned int _channel_id, unsigned int _bandwidth_id, unsigned int _txgain_id);

static int callback(unsigned char * _header,  int _header_valid,
                    unsigned char * _payload, int _payload_valid,
                    void * _userdata)
{
    crdata * p = (crdata*) _userdata;

    // lock internal mutex
    p->num_rx_packets++;
    if (_header_valid && _payload_valid)
        p->num_valid_rx_packets++;
    // unlock internal mutex

#if VERBOSE
    printf("********* callback invoked, %4u/%4u ",
            p->num_valid_rx_packets, p->num_rx_packets);
    if ( !_header_valid ) {
        printf("HEADER CRC FAIL\n");
    } else if ( !_payload_valid ) {
        printf("PAYLOAD CRC FAIL\n");
    }
#endif

    if ( !_header_valid || !_payload_valid )
        return 0;

    // lock mutex
    pthread_mutex_lock(&(p->rx_data_mutex));

    // copy data
    memmove(p->rx_header,  _header , 24);
    memmove(p->rx_payload, _payload, 64);
    p->rx_header_valid  = _header_valid;
    p->rx_payload_valid = _payload_valid;

    // decode packet header
    pm_disassemble_header(p->rx_header, &(p->rx_pm_header));
#if VERBOSE
    printf("packet id: %u\n", p->rx_pm_header.pid);
#endif

    // unlock mutex
    pthread_mutex_unlock(&(p->rx_data_mutex));

    // signal condition (received packet)
    pthread_cond_signal(&(p->rx_data_ready));

    return 0;
}

int main (int argc, char **argv)
{
    srand(time(NULL));

    // create common data object
    crdata data;
    data.mode = OPMODE_SLAVE;
    data.node_id = rand() & 0xffff;
    data.fs_tx = 62.5e3f;
    data.fs_rx = 62.5e3f;

    //
    int d;
    while ((d = getopt(argc,argv,"msi:r:")) != EOF) {
        switch (d) {
        case 'm':
            data.mode = OPMODE_MASTER;
            break;
        case 's':
            data.mode = OPMODE_SLAVE;
            break;
        case 'i':
            data.node_id = atoi(optarg) & 0xffff;
            break;
        case 'r':
            data.fs_tx = atof(optarg);
            data.fs_rx = atof(optarg);
            break;
        default:    /* print help() */  return 0;
        }
    }
    printf("node id: %d\n", data.node_id);

    data.fc = pm_get_channel_frequency(32*4);
    data.tx_gain = 8000;
    data.ack_timeout = 100; // (ms)
    data.ce_sleep = 500;   // (ms)
    data.pm_attempt_timeout = 10;   // number of packets dropped before taking action

    //
    data.num_rx_packets = 0;
    data.num_valid_rx_packets = 0;
    data.num_ack_timeouts = 0;

    // packet header properties
    data.tx_pm_header.do_set_control        = 0;

    data.urx =  usrp_standard_rx::make (0, 256);
    if (data.urx == 0) {
        fprintf (stderr, "Error: usrp_standard_rx::make\n");
        exit (1);
    }

    data.utx =  usrp_standard_tx::make (0, 512);
    if (data.utx == 0) {
        fprintf (stderr, "Error: usrp_standard_tx::make\n");
        exit (1);
    }
 
    // Set Number of channels
    data.urx->set_nchannels(1);
    data.utx->set_nchannels(1);

    // Set symbol rate
    cr_set_tx_symbol_rate(&data, data.fs_tx);
    cr_set_rx_symbol_rate(&data, data.fs_rx);

    // set ack timeout (empirical relationship)
    data.ack_timeout = (unsigned int) (6000.0e3f / data.fs_tx);
    printf("setting ACK timeout to %u ms\n", data.ack_timeout);

    // set packet manager attempt timeout (empirical value)
    data.pm_attempt_timeout = 800 / (data.ack_timeout);
    printf("setting pm attempt timeout to %u\n", data.pm_attempt_timeout);

    // Set other properties
    data.urx->set_pga(0,0);         // adc pga gain
    data.urx->set_mux(0x32103210);  // Board A only
  
    // tx daughterboard
    data.tx_db_id = data.utx->daughterboard_id(0);
    std::cout << "tx db slot 0 : " << usrp_dbid_to_string(data.tx_db_id) << std::endl;
 
    if (data.tx_db_id == USRP_DBID_FLEX_400_TX_MIMO_B) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400_TX_MIMO_B\n");
        data.txdb = new db_flex400_tx_mimo_b(data.utx,0);
    } else {
        printf("use usrp db flex 400 tx MIMO B\n");
        return 0;
    }   
    data.txdb->set_enable(true);

    // rx daughterboard
    data.rx_db_id = data.urx->daughterboard_id(0);
    std::cout << "rx db slot 0 : " << usrp_dbid_to_string(data.rx_db_id) << std::endl;
 
    if (data.rx_db_id == USRP_DBID_FLEX_400_RX_MIMO_B) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400_RX_MIMO_B\n");
        data.rxdb = new db_flex400_rx_mimo_b(data.urx,0);
    } else {
        printf("use usrp db flex 400 rx MIMO B\n");
        return 0;
    }   

    // set the daughterboard gain
    float gmin, gmax, gstep;
    data.txdb->get_gain_range(gmin,gmax,gstep);
    printf("tx: gmin/gmax/gstep: %f/%f/%f\n", gmin,gmax,gstep);
    //data.db->set_gain(gmax);    // note: not a good idea to set to max
    data.rxdb->get_gain_range(gmin,gmax,gstep);
    printf("rx: gmin/gmax/gstep: %f/%f/%f\n", gmin,gmax,gstep);
    //data.db->set_gain(gmax);    // note: not a good idea to set to max

    // set frequency
    cr_set_tx_frequency(&data, data.fc);
    cr_set_rx_frequency(&data, data.fc);

    // enable automatic transmit/receive
    data.txdb->set_auto_tr(true);
    data.rxdb->set_auto_tr(true);

    // initialize mutexes, etc.
    pthread_mutex_init(&(data.tx_data_mutex),NULL);
    pthread_mutex_init(&(data.rx_data_mutex),NULL);
    pthread_mutex_init(&(data.control_mutex),NULL);
    pthread_cond_init(&(data.tx_data_ready),NULL);
    pthread_cond_init(&(data.rx_data_ready),NULL);
    pthread_cond_init(&(data.control_ready),NULL);

    data.radio_active = true;
    data.ce_waiting = false;

    // create thread objects
    void * status;
    pthread_t tx_thread;
    pthread_t rx_thread;
    pthread_t pm_thread;
    pthread_t ce_thread;

    // set thread attributes
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    // create threads
    pthread_create(&tx_thread, &attr, tx_process, (void*)&data);
    pthread_create(&rx_thread, &attr, rx_process, (void*)&data);
    pthread_create(&pm_thread, &attr, pm_process, (void*)&data);
    pthread_create(&ce_thread, &attr, ce_process, (void*)&data);

    // join threads
    pthread_join(tx_thread, &status);
    pthread_join(rx_thread, &status);
    pthread_join(pm_thread, &status);
    pthread_join(ce_thread, &status);

    //
    printf("finished\n");

    // clean up objects
    pthread_mutex_destroy(&(data.tx_data_mutex));
    pthread_mutex_destroy(&(data.rx_data_mutex));
    pthread_mutex_destroy(&(data.control_mutex));
    pthread_cond_destroy(&(data.tx_data_ready));
    pthread_cond_destroy(&(data.rx_data_ready));
    pthread_cond_destroy(&(data.control_ready));
    return 0;
}

void cr_set_tx_frequency(crdata * _p, float _tx_frequency)
{
    // set the daughterboard frequency
    float db_lo_offset = -8e6;
    float db_lo_freq = 0.0f;
    float db_lo_freq_set = _tx_frequency + db_lo_offset;
    _p->txdb->set_db_freq(db_lo_freq_set, db_lo_freq);
    float ddc_freq = _tx_frequency - db_lo_freq;
    _p->utx->set_tx_freq(USRP_CHANNEL, ddc_freq);

    _p->fc = _tx_frequency;
}


void cr_set_rx_frequency(crdata * _p, float _rx_frequency)
{
    // set the daughterboard frequency
    float db_lo_offset = -8e6;
    float db_lo_freq = 0.0f;
    float db_lo_freq_set = _rx_frequency + db_lo_offset;
    _p->rxdb->set_db_freq(db_lo_freq_set, db_lo_freq);
    float ddc_freq = _rx_frequency - db_lo_freq;
    _p->urx->set_rx_freq(USRP_CHANNEL, ddc_freq);
}

void cr_set_tx_symbol_rate(crdata * _p, float _tx_rate)
{
    // check validity
    if (_tx_rate < TX_RATE_MIN) {
        printf("warning: input tx symbol rate (%f) too small; setting to minimum (%f)\n",
            _tx_rate, TX_RATE_MIN);
        _tx_rate = TX_RATE_MIN;
    } else if (_tx_rate > TX_RATE_MAX) {
        printf("warning: input tx symbol rate (%f) too large; setting to maximum (%f)\n",
            _tx_rate, TX_RATE_MAX);
        _tx_rate = TX_RATE_MAX;
    }

    _p->tx_interp = (unsigned int) (32e6f / _tx_rate);

    // interpolator must be multiple of 4
    _p->tx_interp = ((_p->tx_interp) >> 2) << 2;

    // update actual tx symbol rate
    _p->fs_tx = 32e6 / (_p->tx_interp);

    // set value in usrp
    _p->utx->set_interp_rate( _p->tx_interp );

#if VERBOSE
    printf("setting tx symbol rate to %8.2f kHz (interp: %3u)\n",
            _p->fs_tx * 1e-3f, _p->tx_interp);
#endif

    // set ack timeout (empirical relationship)
    _p->ack_timeout = (unsigned int) (6000.0e3f / _p->fs_tx);
#if VERBOSE
    printf("setting ACK timeout to %u ms\n", _p->ack_timeout);
#endif

    // set packet manager attempt timeout (empirical value)
    _p->pm_attempt_timeout = 800 / (_p->ack_timeout);
#if VERBOSE
    printf("setting pm attempt timeout to %u\n", _p->pm_attempt_timeout);
#endif
}

void cr_set_rx_symbol_rate(crdata * _p, float _rx_rate)
{
    // check validity
    if (_rx_rate < RX_RATE_MIN) {
        printf("warning: input rx symbol rate (%f) too small; setting to minimum (%f)\n",
            _rx_rate, RX_RATE_MIN);
        _rx_rate = RX_RATE_MIN;
    } else if (_rx_rate > RX_RATE_MAX) {
        printf("warning: input rx symbol rate (%f) too large; setting to maximum (%f)\n",
            _rx_rate, RX_RATE_MAX);
        _rx_rate = RX_RATE_MAX;
    }

    _p->rx_decim = (unsigned int) (16e6f / _rx_rate);

    // decimator must be multiple of 2
    _p->rx_decim = ((_p->rx_decim) >> 1) << 1;

    // update actual rx symbol rate
    _p->fs_rx = 16e6 / (_p->rx_decim);

    // set value in usrp
    _p->urx->set_decim_rate( _p->rx_decim );

#if VERBOSE
    printf("setting rx symbol rate to %8.2f kHz (decim:  %3u)\n",
            _p->fs_rx * 1e-3f, _p->rx_decim);
#endif
}

void cr_set_tx_gain(crdata * _p, float _tx_gain)
{
    unsigned short gain;
    if (_tx_gain < 10.0f) {
        printf("warning: cr_set_tx_gain(), gain setting is below minimum\n");
        gain = 10;
    } else if (_tx_gain > 16000.0f) {
        printf("warning: cr_set_tx_gain(), gain setting exceeds maximum\n");
        gain = 16000;
    } else {
        gain = (unsigned short) _tx_gain;
    }

    _p->tx_gain = gain;
}

void cr_set_parameters_by_id(crdata * _p, unsigned int _channel_id, unsigned int _bandwidth_id, unsigned int _txgain_id)
{
    // set channel
    if (_p->channel_id != _channel_id) {
        _p->channel_id = _channel_id;
        float frequency = pm_get_channel_frequency(_p->channel_id);
#if VERBOSE
        printf("*** CONTROL : switching to channel %3u (%8.4f MHz)\n",
            _p->channel_id, frequency*1e-6f);
#endif

        cr_set_tx_frequency(_p, frequency);
        cr_set_rx_frequency(_p, frequency);
    }

    if (_p->bandwidth_id != _bandwidth_id) {
        _p->bandwidth_id = _bandwidth_id;
        float bandwidth = pm_get_bandwidth(_p->bandwidth_id);
#if VERBOSE
        printf("*** CONTROL : node switching to bandwidth %3u (%8.4f kHz)\n",
            _p->bandwidth_id, bandwidth*1e-3f);
#endif

        cr_set_tx_symbol_rate(_p, bandwidth);
        cr_set_rx_symbol_rate(_p, bandwidth);
    }

    if (_p->txgain_id != _txgain_id) {
        _p->txgain_id = _txgain_id;
        float txgain = pm_get_txgain(_p->txgain_id);
#if VERBOSE
        printf("*** CONTROL : node switching to txgain %3u (%8.2f)\n",
                _p->txgain_id, txgain);
#endif

        cr_set_tx_gain(_p, txgain);
    }

}

void * tx_process(void*userdata)
{
    crdata * p = (crdata*) userdata;

    // create buffer
    const int tx_buf_len = 2*2*2048;
    short tx_buf[tx_buf_len];

    // create interpolator
    //unsigned int k=2; // samples per symbol
    unsigned int m=2; // delay
    float beta=0.7f;  // excess bandwidth factor
    resamp2_crcf interpolator = resamp2_crcf_create(13,0.0f,40.0f);
    //unsigned int block_size = tx_buf_len/2;     // number of cplx samp / tx
    //unsigned int num_blocks = 2048/block_size;  // number of cplx blocks / fr.
    //unsigned int num_flush = 16; // number of blocks to use for flushing (off time)
    std::complex<float> interp_buffer[2*2048];
    //std::complex<float> * block_ptr;

    // framing
    std::complex<float> frame[2048];
    framegen64 framegen = framegen64_create(m,beta);

    p->utx->start();        // Start data transfer
    printf("usrp tx transfer started\n");

    unsigned int n;
    short I, Q;
    bool underrun;
    unsigned int num_underruns;
    while (p->radio_active) {
        // wait for signal condition
        //printf("tx: waiting for data\n");
        pthread_mutex_lock(&(p->tx_data_mutex));
        pthread_cond_wait(&(p->tx_data_ready),&(p->tx_data_mutex));
        //printf("tx: received tx_data_ready signal\n");

        // generate the frame
        framegen64_execute(framegen, p->tx_header, p->tx_payload, frame);
        pthread_mutex_unlock(&(p->tx_data_mutex));
        
        // TODO: flush the frame

        // run interpolator
        for (n=0; n<2048; n++) {
            resamp2_crcf_interp_execute(interpolator,
                frame[n], &interp_buffer[2*n]);
        }

        // prepare data
        for (n=0; n<4096; n++) {
            I = (short) (interp_buffer[n].real() * p->tx_gain);
            Q = (short) (interp_buffer[n].imag() * p->tx_gain);

            tx_buf[2*n+0] = host_to_usrp_short(I);
            tx_buf[2*n+1] = host_to_usrp_short(Q);
        }

        // write data
        //printf("tx: writing to usrp...\n");
        int rc = p->utx->write(tx_buf, tx_buf_len*sizeof(short), &underrun); 
                    
        if (underrun) {
            printf ("USRP tx underrun\n");
            num_underruns++;
        }

        if (rc < 0) {
            printf("error occurred with USRP\n");
            exit(0);
        } else if (rc != tx_buf_len*sizeof(short)) {
            printf("error: did not write proper length\n");
            exit(0);
        }

    }
 
    // stop data transfer
    p->utx->stop();
    printf("usrp tx transfer stopped\n");

    // clean up allocated memory
    framegen64_destroy(framegen);
    resamp2_crcf_destroy(interpolator);

    // exit thread
    pthread_exit(NULL);
}

void * rx_process(void*userdata)
{
    crdata * p = (crdata*) userdata;

    // create buffer
    const int    rx_buf_len = 512/2; // Should be multiple of 512 Bytes
    short  rx_buf[rx_buf_len];
    bool overrun;
    int num_overruns=0;

    // framing
    unsigned int m=2;
    float beta=0.7f;
    framesync64 framesync = framesync64_create(m,beta,callback,(void*)p);

    // create decimator
    resamp2_crcf decimator = resamp2_crcf_create(13,0.0f,40.0f);
    std::complex<float> buffer[rx_buf_len/2];
    std::complex<float> decim_out[rx_buf_len/4];
 
    p->urx->start();        // Start data transfer
    printf("usrp rx transfer started\n");

    unsigned int reset=0;
    int n;
    while (p->radio_active) {
        // read data
        float g = usrp_rx_gain_correction(p->rx_decim);
        p->urx->read(rx_buf, rx_buf_len*sizeof(short), &overrun); 
            
        if (overrun) {
            printf ("USRP Rx Overrun\n");
            num_overruns++;
        }
 
        // convert to float
        std::complex<float> sample;
        for (n=0; n<rx_buf_len; n+=2) {
            sample.real() = (float) ( rx_buf[n+0]) * g * 0.01f;
            sample.imag() = (float) (-rx_buf[n+1]) * g * 0.01f;

            buffer[n/2] = sample;
        }

        // run decimator
        for (n=0; n<rx_buf_len/2; n+=2) {
            resamp2_crcf_decim_execute(decimator, &buffer[n], &decim_out[n/2]);
        }

        // run through frame synchronizer
        framesync64_execute(framesync, decim_out, rx_buf_len/4);

        // periodically reset the synchronizer
        reset++;
        if (reset == 10000) {
#if VERBOSE
            printf("resetting synchronizer\n");
#endif
            reset = 0;
            framesync64_reset(framesync);
        }
    }

    p->urx->stop();  // Stop data transfer
    printf("usrp rx transfer stopped\n");


    // clean up memory allocation
    framesync64_destroy(framesync);
    resamp2_crcf_destroy(decimator);

    pthread_exit(NULL);
}

void * pm_process(void*userdata)
{
    crdata * p = (crdata*) userdata;

    printf("pm_process started, mode : %s\n", p->mode == OPMODE_MASTER ?
            "master" : "slave");

    unsigned int pid=0, pm_attempt, channel=0;
    //float channel_frequency;
    while (p->radio_active) {

        pid = (pid+1) & 0xffff;
        p->packet_received = false;
        pm_attempt = 0;

        switch (p->mode) {
        case OPMODE_MASTER:
            // set control parameters
            p->tx_pm_header.do_set_control = 0;

            // continue transmitting until packet is received
            do {

                // lock control mutex
                usleep(500);  // sleep a short time to give ce a chance to lock control mutex
                pthread_mutex_lock(&(p->control_mutex));

#if VERBOSE
                printf("transmitting packet %u (attempt %u)\n", pid, pm_attempt);
#endif
                if (p->tx_pm_header.do_set_control) {
#if VERBOSE
                    printf("***** sending control signal: ch: %u, bw: %u, gain: %u, bch0: %u\n",
                        p->tx_pm_header.ctrl_channel,
                        p->tx_pm_header.ctrl_bandwidth,
                        p->tx_pm_header.ctrl_txgain,
                        p->tx_pm_header.ctrl_bch0);
#endif
                }
                pm_send_data_packet(p,pid);

                p->packet_received = pm_wait_for_ack_packet(p,pid);
                if (!p->packet_received)
                    p->num_ack_timeouts++;

#if VERBOSE
                if (p->packet_received && p->tx_pm_header.do_set_control)
                    printf("****** control received ack\n");
#endif

                pm_attempt++;

                if ((pm_attempt % (p->pm_attempt_timeout))==0) {
                    // disable control
                    p->tx_pm_header.do_set_control = 0;

                    // change frequency (rendezvous channel)
                    channel = 32*(rand() % 8);
                    cr_set_parameters_by_id(p, channel, 0, 128);
                    
                };

                pthread_mutex_unlock(&(p->control_mutex));
                if (p->ce_waiting) {
                    //printf("  ==> pm sees ce is waiting; signaling condition\n");
                    pthread_cond_signal(&(p->control_ready));
                    p->ce_waiting = false;
                }

            } while (!p->packet_received && p->radio_active);

            if (p->tx_pm_header.do_set_control) {
                // switch parameters
                cr_set_parameters_by_id(p,
                    p->tx_pm_header.ctrl_channel,
                    p->tx_pm_header.ctrl_bandwidth,
                    p->tx_pm_header.ctrl_txgain);
            }

            break;
        case OPMODE_SLAVE:
            // wait until packet is received

            do {
#if VERBOSE
                printf("slave waiting for data packet %u...\n",pid);
#endif
                p->packet_received = pm_wait_for_data_packet(p,pid);
                if (!p->packet_received)
                    p->num_ack_timeouts++;
                pm_attempt++;

                if ((pm_attempt % (p->pm_attempt_timeout))==0) {
                    // change frequency (rendezvous channel)
                    channel = 32*(rand() % 8);
                    cr_set_parameters_by_id(p, channel, 0, 128);
                };

            } while (!p->packet_received && p->radio_active);

            pm_disassemble_header(p->rx_header, &(p->rx_pm_header));

#if VERBOSE
            printf("pm: packet id: %u\n", p->rx_pm_header.pid);
#endif

            // send ACK
            pm_send_ack_packet(p,p->rx_pm_header.pid);

            // set control parameters
            if (p->rx_pm_header.do_set_control) {
#if VERBOSE
                printf("***** receiving control signal: ch: %u, bw: %u, gain: %u, bch0: %u\n",
                    p->rx_pm_header.ctrl_channel,
                    p->rx_pm_header.ctrl_bandwidth,
                    p->rx_pm_header.ctrl_txgain,
                    p->rx_pm_header.ctrl_bch0);

                printf("*** sending extra ACK\n");
#endif
                pm_send_ack_packet(p,p->rx_pm_header.pid);

                usleep(2*1000*(p->ack_timeout));

                cr_set_parameters_by_id(p,
                    p->rx_pm_header.ctrl_channel,
                    p->rx_pm_header.ctrl_bandwidth,
                    p->rx_pm_header.ctrl_txgain);
            }

            break;
        default:
            printf("error: pm_process(), unknown operating mode: %u\n", p->mode);
            exit(1);
        }
    }

    pthread_exit(NULL);
}

void pm_send_data_packet(crdata * p, unsigned int pid)
{

    pthread_mutex_lock(&(p->tx_data_mutex));

    unsigned int i;
    //for (i=0; i<24; i++) p->tx_header[i]    = rand()%256;
    p->tx_pm_header.pid = pid;
    p->tx_pm_header.type = PACKET_TYPE_DATA;

    pm_assemble_header(p->tx_pm_header, p->tx_header);
    for (i=0; i<64; i++) p->tx_payload[i]   = rand()%256;

    pthread_mutex_unlock(&(p->tx_data_mutex));
    pthread_cond_signal(&(p->tx_data_ready));
}

void pm_send_ack_packet(crdata * p, unsigned int pid)
{
#if VERBOSE
    printf("pm: transmitting ack on packet %u\n", pid);
#endif

    pthread_mutex_lock(&(p->tx_data_mutex));

    p->tx_pm_header.pid = pid;
    p->tx_pm_header.type = PACKET_TYPE_ACK;
    pm_assemble_header(p->tx_pm_header, p->tx_header);

    pthread_cond_signal(&(p->tx_data_ready));
    pthread_mutex_unlock(&(p->tx_data_mutex));

}

bool pm_wait_for_ack_packet(crdata * p, unsigned int pid)
{
    // wait until packet is received

    // set timeout variables
    int rc;
    struct timespec ts;
    struct timeval  tp;
    rc = gettimeofday(&tp,NULL);
    ts.tv_sec  = tp.tv_sec;
    ts.tv_nsec = (tp.tv_usec + 1000*(p->ack_timeout)) * 1000;

    while (ts.tv_nsec > 1000000000) {
        ts.tv_nsec -= 1000000000;
        ts.tv_sec += 1;
    }

    pthread_mutex_lock(&(p->rx_data_mutex));
    rc = pthread_cond_timedwait(&(p->rx_data_ready),&(p->rx_data_mutex),&ts);
#if VERBOSE
    printf("pm: received rx_data_ready signal, rc = %d\n", rc);
#endif
    pthread_mutex_unlock(&(p->rx_data_mutex));

    if (rc != 0) {
#if VERBOSE
        printf("  ==> timeout\n");
#endif
        return false;
    }

    // check received packet
    if ( p->rx_pm_header.type != PACKET_TYPE_ACK ) {
#if VERBOSE
        printf("  ==> wrong packet type (expecing ACK)\n");
#endif
        return false;
    } else if ( p->rx_pm_header.pid != pid ) {
#if VERBOSE
        printf("  ==> wrong packet id (received %u, expected %u)\n",  p->rx_pm_header.pid, pid);
#endif
        return false;
    }

#if VERBOSE
    printf("pm: received ack on packet %u\n", pid);
#endif
    return true;
}
 
bool pm_wait_for_data_packet(crdata * p, unsigned int pid)
{
    // wait until packet is received

    // set timeout variables
    int rc;
    struct timespec ts;
    struct timeval  tp;
    rc = gettimeofday(&tp,NULL);
    ts.tv_sec  = tp.tv_sec;
    ts.tv_nsec = (tp.tv_usec + 1000*(p->ack_timeout)) * 1000;

    while (ts.tv_nsec > 1000000000) {
        ts.tv_nsec -= 1000000000;
        ts.tv_sec += 1;
    }

    pthread_mutex_lock(&(p->rx_data_mutex));
    rc = pthread_cond_timedwait(&(p->rx_data_ready),&(p->rx_data_mutex),&ts);
#if VERBOSE
    printf("pm: received rx_data_ready signal, rc = %d\n", rc);
#endif
    pthread_mutex_unlock(&(p->rx_data_mutex));

    if (rc != 0) {
#if VERBOSE
        printf("  ==> timeout\n");
#endif
        return false;
    }

    // check received packet
    if ( p->rx_pm_header.type != PACKET_TYPE_DATA ) {
//#if VERBOSE
        printf("  ==> wrong packet type (expecing DATA)\n");
//#endif
        return false;
    }

#if VERBOSE
    printf("pm: received data packet %u\n", pid);
#endif

    return true;
}
 
void pm_assemble_header(pm_header _h, unsigned char * _header)
{
    _header[0] = (_h.src0 >> 8) & 0x00ff;
    _header[1] = (_h.src0     ) & 0x00ff;

    _header[2] = (_h.src1 >> 8) & 0x00ff;
    _header[3] = (_h.src1     ) & 0x00ff;

    _header[4] = (_h.dst0 >> 8) & 0x00ff;
    _header[5] = (_h.dst0     ) & 0x00ff;

    _header[6] = (_h.dst1 >> 8) & 0x00ff;
    _header[7] = (_h.dst1     ) & 0x00ff;

    _header[8] = (_h.pid >> 8)  & 0x00ff;
    _header[9] = (_h.pid     )  & 0x00ff;

    _header[10] = _h.type & 0x00ff;

    // control

    _header[14] = _h.do_set_control;
    _header[15] = _h.ctrl_channel;
    _header[16] = _h.ctrl_bandwidth;
    _header[17] = _h.ctrl_txgain;
    _header[18] = _h.ctrl_bch0;
}

void pm_disassemble_header(unsigned char * _header, pm_header * _h)
{
    _h->src0 = (_header[0] << 8) | (_header[1]);
    _h->src1 = (_header[2] << 8) | (_header[3]);
    _h->dst0 = (_header[4] << 8) | (_header[5]);
    _h->dst1 = (_header[6] << 8) | (_header[7]);

    _h->pid  = (_header[8] << 8) | (_header[9]);

    _h->type = _header[10];

    // control
    _h->do_set_control  = _header[14] & 0x01;
    _h->ctrl_channel    = _header[15];
    _h->ctrl_bandwidth  = _header[16];
    _h->ctrl_txgain     = _header[17];
    _h->ctrl_bch0       = _header[18];

}

void * ce_process(void*userdata)
{
    crdata * p = (crdata*) userdata;

    float throughput;
    bool run_cognition_cycle;
    //unsigned int i;
    //for (i=0; i<100; i++) {
    while (true) {
        // sleep for several seconds
        usleep((p->ce_sleep)*1000);

        if ((rand()%4)==0 && p->mode == OPMODE_MASTER)
            run_cognition_cycle = true;
        else
            run_cognition_cycle = false;

        // measure throughput
        throughput = 64*8*(p->num_valid_rx_packets)/((p->ce_sleep)/1000.0f);
        printf("ce %1s %8.3f kb/s, packets [%4u / %4u] %4u timeout(s), fc : %8.2fMHz, fd : %8.2fkHz\n",
                run_cognition_cycle ? "*" : " ",
                throughput*1e-3,
                p->num_valid_rx_packets,
                p->num_rx_packets,
                p->num_ack_timeouts,
                p->fc * 1e-6f,
                p->fs_tx * 1e-3f);

        p->num_rx_packets = 0;
        p->num_valid_rx_packets = 0;
        p->num_ack_timeouts = 0;

        if (p->mode != OPMODE_MASTER)
            continue;

        // TODO: lock internal mutex
        pthread_mutex_lock(&(p->control_mutex));

        p->ce_waiting = true;
        pthread_cond_wait(&(p->control_ready),&(p->control_mutex));

        // randomly send control signal
        if (run_cognition_cycle) {
            p->tx_pm_header.do_set_control  = 1;

            //
#if VERBOSE
            printf("## EXECUTE COGNITION CYCLE ##\n");
#endif
            p->tx_pm_header.ctrl_channel    = rand()%256;
            p->tx_pm_header.ctrl_bandwidth  = rand()%256;
            p->tx_pm_header.ctrl_txgain     = 100 + (rand()%128);
            p->tx_pm_header.ctrl_bch0       = 4;
        }

        // TODO: unlock internal mutex
        pthread_mutex_unlock(&(p->control_mutex));
    }

    // lock mutex
    p->radio_active = false;
    // unlock mutex

    pthread_exit(NULL);
}

