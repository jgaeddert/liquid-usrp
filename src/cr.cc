//
// Liquid cognitive radio prototype
//
 
#include <math.h>
#include <iostream>
#include <complex>
#include <pthread.h>
#include <liquid/liquid.h>

#include "usrp_standard.h"
#include "usrp_prims.h"
#include "usrp_dbid.h"
#include "usrp_bytesex.h"
#include "flex.h"
 
/*
 SAMPLES_PER_READ :Each sample is consists of 4 bytes (2 bytes for I and 
 2 bytes for Q. Since the reading length from USRP should be multiple of 512 
 bytes see "usrp_basic.h", then we have to read multiple of 128 samples each 
 time (4 bytes * 128 sample = 512 bytes)  
 */
#define SAMPLES_PER_READ    (512)       // Must be a multiple of 128
#define USRP_CHANNEL        (0)
 
typedef struct crdata {
    // cognitive radio parameters
    float fc;               // carrier frequency
    unsigned int fd_tx;     // tx data rate (interp)
    unsigned int fd_rx;     // rx data rate (decim)
    unsigned short tx_gain; // from 0 to 20,000

    // front end objects
    usrp_standard_tx * utx;
    usrp_standard_rx * urx;
    int dbid;
    db_base * db;

    // threading
    pthread_mutex_t rx_mutex;
    pthread_mutex_t control_mutex;
    pthread_mutex_t internal_mutex;
    pthread_cond_t  tx_data_ready;

    // data buffers
    unsigned char tx_header[24];
    unsigned char tx_payload[64];
    bool tx_header_crc_pass;
    bool tx_payload_crc_pass;

    unsigned char rx_header[24];
    unsigned char rx_payload[64];
    bool rx_header_crc_pass;
    bool rx_payload_crc_pass;
};

void * tx_process(void*);   // transmitter
void * rx_process(void*);   // receiver
void * ce_process(void*);   // cognitive engine
 
void usrp_set_frequency(usrp_standard_rx * _urx, db_base * _db, float _frequency);

static int callback(unsigned char * _header,  int _header_valid,
                    unsigned char * _payload, int _payload_valid)
{
    std::cout << "********* callback invoked, ";// << std::endl;
    if ( !_header_valid ) {
        printf("HEADER CRC FAIL\n");
    } else if ( !_payload_valid ) {
        printf("PAYLOAD CRC FAIL\n");
    } else {
        printf("packet id: %u\n", (unsigned int ) _header[0]);
    }
    return 0;
}

int main (int argc, char **argv)
{
    // create common data object
    crdata data;
    data.fc = 462e6;

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

    // Set other properties
    data.urx->set_pga(0,0);         // adc pga gain
    data.urx->set_mux(0x32103210);  // Board A only
 
    // daughterboard
    data.dbid = data.urx->daughterboard_id(0);
    std::cout << "rx db slot 0 : " << usrp_dbid_to_string(data.dbid) << std::endl;
 
    if (data.dbid == USRP_DBID_FLEX_400_RX_MIMO_B) {
        printf("usrp daughterboard: USRP_DBID_FLEX_400_RX_MIMO_B\n");
        data.db = new db_flex400_rx_mimo_b(data.urx,0);
    } else {
        printf("use usrp db flex 400 rx MIMO B\n");
        return 0;
    }   
    data.db->set_enable(true);

    // set the daughterboard gain
    float gmin, gmax, gstep;
    data.db->get_gain_range(gmin,gmax,gstep);
    printf("gmin/gmax/gstep: %f/%f/%f\n", gmin,gmax,gstep);
    //data.db->set_gain(gmax);    // note: not a good idea to set to max

    // set frequency
    usrp_set_frequency(data.urx, data.db, data.fc);

    // initialize mutexes, etc.
    pthread_mutex_init(&(data.rx_mutex),NULL);
    pthread_mutex_init(&(data.internal_mutex),NULL);
    pthread_mutex_init(&(data.control_mutex),NULL);
    pthread_cond_init(&(data.tx_data_ready),NULL);

    // create thread objects
    void * status;
    pthread_t tx_thread;
    pthread_t rx_thread;
    pthread_t ce_thread;

    // set thread attributes
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    // create threads
    pthread_create(&tx_thread, &attr, tx_process, (void*)&data);
    pthread_create(&rx_thread, &attr, rx_process, (void*)&data);
    pthread_create(&ce_thread, &attr, ce_process, (void*)&data);

    // join threads
    pthread_join(tx_thread, &status);
    pthread_join(rx_thread, &status);
    pthread_join(ce_thread, &status);

    //
    printf("finished\n");

    // clean up objects
    pthread_mutex_destroy(&(data.rx_mutex));
    pthread_mutex_destroy(&(data.internal_mutex));
    pthread_mutex_destroy(&(data.control_mutex));
    pthread_cond_destroy(&(data.tx_data_ready));
    return 0;
}

void usrp_set_frequency(usrp_standard_rx * _urx, db_base * _db, float _frequency)
{
    // set the daughterboard frequency
    float db_lo_offset = -8e6;
    float db_lo_freq = 0.0f;
    float db_lo_freq_set = _frequency + db_lo_offset;
    _db->set_db_freq(db_lo_freq_set, db_lo_freq);
    float ddc_freq = _frequency - db_lo_freq;
    _urx->set_rx_freq(USRP_CHANNEL, ddc_freq);
}


void * tx_process(void*userdata)
{
    crdata * p = (crdata*) userdata;

    // create buffer
    const int tx_buf_len = 2*2*2048;
    short tx_buf[tx_buf_len];

    // create interpolator
    unsigned int k=2; // samples per symbol
    unsigned int m=3; // delay
    float beta=0.7f;  // excess bandwidth factor
    resamp2_crcf interpolator = resamp2_crcf_create(37);
    unsigned int block_size = tx_buf_len/2;     // number of cplx samp / tx
    unsigned int num_blocks = 2048/block_size;  // number of cplx blocks / fr.
    unsigned int num_flush = 16; // number of blocks to use for flushing (off time)
    std::complex<float> interp_buffer[2*2048];
    std::complex<float> * block_ptr;

    // framing
    std::complex<float> frame[2048];
    framegen64 framegen = framegen64_create(m,beta);

    p->utx->start();        // Start data transfer
    printf("usrp tx transfer started\n");

    unsigned int i, n;
    short I, Q;
    bool underrun;
    unsigned int num_underruns;
    for (i=0; i<1000000; i++) {
        // wait for signal condition
        printf("tx: waiting for data\n");
        pthread_mutex_lock(&(p->internal_mutex));
        pthread_cond_wait(&(p->tx_data_ready),&(p->internal_mutex));

        // lock receiver mutex
        pthread_mutex_lock(&(p->rx_mutex));

        // generate the frame
        framegen64_execute(framegen, p->tx_header, p->tx_payload, frame);
        
        pthread_mutex_unlock(&(p->internal_mutex));

        // TODO: flush the frame

        // run interpolator
        for (n=0; n<2048; n++) {
            resamp2_crcf_interp_execute(interpolator,
                frame[n], &interp_buffer[2*n]);
        }

        // prepare data
        for (n=0; n<4096; n++) {
            I = (short) (interp_buffer[n].real() * 1000);
            Q = (short) (interp_buffer[n].imag() * 1000);

            tx_buf[2*n+0] = host_to_usrp_short(I);
            tx_buf[2*n+1] = host_to_usrp_short(Q);
        }

        // write data
        //utx->write(&buf, bufsize, &underrun); 
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

        // unlock receiver mutex
        pthread_mutex_unlock(&(p->rx_mutex));
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
    const int    rx_buf_len = 512*2; // Should be multiple of 512 Bytes
    short  rx_buf[rx_buf_len];
    bool overrun;
    int num_overruns=0;

    // framing
    unsigned int m=3;
    float beta=0.7f;
    framesync64 framesync = framesync64_create(m,beta,callback);

    // create decimator
    resamp2_crcf decimator = resamp2_crcf_create(37);
    std::complex<float> buffer[rx_buf_len/2];
    std::complex<float> decim_out[rx_buf_len/4];
 
    p->urx->start();        // Start data transfer
    printf("usrp rx transfer started\n");

    int n;
    while (true) {
        // lock receiver mutex
        //pthread_mutex_lock(&(p->rx_mutex));

        // read data
        p->urx->read(rx_buf, rx_buf_len*sizeof(short), &overrun); 
            
        if (overrun) {
            printf ("USRP Rx Overrun\n");
            num_overruns++;
        }
 
        // convert to float
        std::complex<float> sample;
        for (n=0; n<rx_buf_len; n+=2) {
            sample.real() = (float) ( rx_buf[n+0]) * 0.01f;
            sample.imag() = (float) (-rx_buf[n+1]) * 0.01f;

            buffer[n/2] = sample;
        }

        // run decimator
        for (n=0; n<rx_buf_len/2; n+=2) {
            resamp2_crcf_decim_execute(decimator, &buffer[n], &decim_out[n/2]);
        }

        // run through frame synchronizer
        framesync64_execute(framesync, decim_out, rx_buf_len/4);

        // unlock receiver mutex
        //pthread_mutex_unlock(&(p->rx_mutex));
    }

    p->urx->stop();  // Stop data transfer
    printf("usrp rx transfer stopped\n");


    // clean up memory allocation
    framesync64_destroy(framesync);
    resamp2_crcf_destroy(decimator);

    pthread_exit(NULL);
}

void * ce_process(void*userdata)
{
    crdata * p = (crdata*) userdata;

    pthread_exit(NULL);
}

