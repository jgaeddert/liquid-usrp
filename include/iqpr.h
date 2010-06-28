//
// liquid packet radio
//

#ifndef __IQPR_H__
#define __IQPR_H__

#ifdef __cplusplus
extern "C" {
#   define LIQUID_USE_COMPLEX_H 0
#else
#   define LIQUID_USE_COMPLEX_H 1
#endif /* __cplusplus */

typedef struct iqpr_s * iqpr;

iqpr iqpr_create();
void iqpr_destroy(iqpr _q);
void iqpr_print(iqpr _q);

// configure properties
//
// properties available:
//  f   :   center frequency
//  b   :   bandwidth (symbol rate)
//  n   :   payload length (bytes)
//  m   :   mod. scheme: psk, dpsk, ask, qam, apsk...
//  p   :   mod. depth: 1,2,...,8
//  r   :   ramp up/dn length
//  z   :   phasing length
//  c   :   fec coding scheme (inner)
//  k   :   fec coding scheme (outer)
//  g   :   tx gain [dB]
//  w   :   backoff time (us)
void iqpr_configure(iqpr _q, void * _props);

//
//  s   :   signal-to-noise ratio (estimate) [dB]
//  r   :   data rate
//  p   :   packets received
//  h   :   valid headers received
//  v   :   valid packets received
//  g   :   rssi
//  c   :   cpu usage
void iqpr_get_internals(iqpr _q, void * _props);

#define IQPR_UDP    0   // UDP-like packet (don't wait for 'ACK')
#define IQPR_TCP    1   // TCP-like packet (wait for 'ACK')

// iqpr_send()
void iqpr_send(iqpr _q,
               unsigned int _node_id,
               unsigned char * _data,
               unsigned int _n,
               int _packet_type);

// iqpr_recv()
void iqpr_recv(iqpr _q);

// iqpr_select()
void iqpr_select(iqpr _q);

// 
// internal methods
//

void iqpr_start_threads(iqpr _q);
void iqpr_stop_threads(iqpr _q);

//void iqpr_tx_encode_header(iqpr _q);
//void iqpr_rx_decode_header(iqpr _q);

// threads
void * iqpr_tx_process(void * _q);
void * iqpr_rx_process(void * _q);

// callback
int iqpr_callback(unsigned char * _rx_header,
                  int _rx_header_valid,
                  unsigned char * _rx_payload,
                  unsigned int _rx_payload_len,
                  framesyncstats_s _stats,
                  void * _userdata);


#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif // __IQPR_H__

