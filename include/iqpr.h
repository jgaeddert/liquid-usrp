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

iqpr iqpr_create(unsigned int _node_id);
void iqpr_destroy(iqpr _q);
void iqpr_print(iqpr _q);

// open connection to remote node
//  _q          :   iqpr packetizer object
//  _node_id    :   identification of destination node
//  _link_type  :   tcp/udp/...
//
// returns:
//  -1          :   link cannot be established (timeout)
//   0          :   link cannot be established (connection refused)
//   1          :   link established
int iqpr_open_connection(iqpr _q,
                         unsigned int _node_id,
                         unsigned int _link_type);

// close connection to remote node
//  _q          :   iqpr packetizer object
//  _node_id    :   identification of destination node
//
// returns:
//  -1          :   link to destination node does not exist
//   0          :   link cannot be established (refused)
//   1          :   link established
int iqpr_close_connection(iqpr _q, unsigned int _node_id);

// print connections
void iqpr_print_connections(iqpr _q);

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

#define IQPR_UDP        0       // UDP-like packet (don't wait for 'ACK')
#define IQPR_TCP        1       // TCP-like packet (wait for 'ACK')
#define IQPR_BROADCAST  2       // broadcast packet

// internal definitions
#define IQPR_CONNECT    20      // request link connection
#define IQPR_DISCONNECT 21      // request link disconnection

// send data packet to _node_id
int iqpr_send(iqpr _q,
              unsigned int _node_id,
              unsigned char * _data,
              unsigned int _n,
              int _packet_type);

// wait to receive packet
int iqpr_recv(iqpr _q,
              unsigned int * _node_id,
              unsigned char * _data,
              unsigned int * _n,
              int * _packet_type);

// iqpr_select()
int iqpr_select(iqpr _q);

// 
// internal methods
//

// start/stop transmit/receive threads
void iqpr_start_threads(iqpr _q);
void iqpr_stop_threads(iqpr _q);

// threads
void * iqpr_tx_process(void * _q);
void * iqpr_rx_process(void * _q);

// receiver callback function
int iqpr_callback(unsigned char * _rx_header,
                  int _rx_header_valid,
                  unsigned char * _rx_payload,
                  unsigned int _rx_payload_len,
                  framesyncstats_s _stats,
                  void * _userdata);

void iqpr_callback_csma_lock(void * _userdata);
void iqpr_callback_csma_unlock(void * _userdata);


// iqpr header 
struct iqpr_header_s {
    unsigned int packet_id;     // header[0:1]
    unsigned int payload_len;   // header[2:3]
    fec_scheme fec0;            // header[4]
    fec_scheme fec1;            // header[5]
    unsigned int node_id_src;   // header[6]
    unsigned int node_id_dst;   // header[7]
    unsigned int packet_type;   // header[8]
};

// print header to screen
void iqpr_header_print(struct iqpr_header_s _header);

// encode header into buffer
void iqpr_header_encode(struct iqpr_header_s _header,
                        unsigned char * _header_data);

// decode header from buffer
void iqpr_header_decode(unsigned char * _header_data,
                        struct iqpr_header_s * _header);

#if 0
// 
// iqpr connection
//
struct iqpr_connection_s {
    unsigned int node_id_dst;   // destination node id
    unsigned char * buffer;
    flexframegenprops_s fgprops;

    // status variables
    int ack_waiting;            // waiting for ack?
    unsigned int ack_packet_id; // packet id for ack

    unsigned int num_packets_received;
    unsigned int num_valid_headers_received;
    unsigned int num_valid_packets_received;
    unsigned int num_bytes_received;
};

// initialize structure
void iqpr_connection_init(struct iqpr_connection_s * _c);
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif // __IQPR_H__

