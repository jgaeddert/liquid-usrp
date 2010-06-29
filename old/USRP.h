/****************************************************************************

Copyright 2005, 2006, 2007 Virginia Polytechnic Institute and State University

This file is part of the OSSIE USRP Device.

OSSIE USRP Device is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

OSSIE USRP Device is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with OSSIE USRP Device; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


****************************************************************************/

#include <vector>

#include <omnithread.h>

#include "usrp_standard.h"
#include "usrp_dbid.h"
#include "usrp_prims.h"

#include "ossie/cf.h"
#include "ossie/PortTypes.h"

#include "standardinterfaces/complexShort_u.h"
#include "standardinterfaces/complexShort_p.h"
#include "standardinterfaces/Radio_Control_p.h"

#include "ossie/Device_impl.h"

#include "flex.h"
#include "dbsrx.h"
#include "tvrx.h"

// Definitions for provides ports
class USRP_i;

class USRP_RX_Control_i : public standardInterfaces_i::RX_Control_p

{

  public:
    USRP_RX_Control_i(USRP_i *_usrp, const char* _name, const char* _domain);
    void set_number_of_channels(CORBA::ULong num);
    void get_number_of_channels(CORBA::ULong &num);

    void get_gain_range(CORBA::ULong channel, CORBA::Float &gmin, CORBA::Float &gmax, CORBA::Float &gstep);
    void set_gain(CORBA::ULong channel, CORBA::Float gain);
    void get_gain(CORBA::ULong channel, CORBA::Float &gain);

    void get_frequency_range(CORBA::ULong channel, CORBA::Float &fmin, CORBA::Float &fmax, CORBA::Float &fstep);
    void set_frequency(CORBA::ULong channel, CORBA::Float f);
    void get_frequency(CORBA::ULong channel, CORBA::Float &f);

    void start(CORBA::ULong channel);
    void stop(CORBA::ULong channel);

    void set_values(const CF::Properties &values);

    void set_decimation_rate(CORBA::ULong channel, CORBA::ULong M);
    void get_decimation_range(CORBA::ULong channel, CORBA::ULong &dmin, CORBA::ULong &dmax, CORBA::ULong &dstep);

    void set_data_packet_size(CORBA::ULong channel, CORBA::ULong N);

 private:
    USRP_RX_Control_i();  // No default constructor
    USRP_RX_Control_i(const USRP_RX_Control_i &); // No copying
 
    USRP_i *usrp;

    float db_lo_freq;
    float db_lo_offset;
    bool lo_locked;

};

class USRP_TX_Control_i : public standardInterfaces_i::TX_Control_p

{
  public:
    USRP_TX_Control_i(USRP_i *_usrp, const char* _name, const char* _domain);
    void set_number_of_channels(CORBA::ULong num);
    void get_number_of_channels(CORBA::ULong &num);

    void get_gain_range(CORBA::ULong channel, CORBA::Float &gmin, CORBA::Float &gmax, CORBA::Float &gstep);
    void set_gain(CORBA::ULong channel, CORBA::Float gain);
    void get_gain(CORBA::ULong channel, CORBA::Float &gain);

    void get_frequency_range(CORBA::ULong channel, CORBA::Float &fmin, CORBA::Float &fmax, CORBA::Float &fstep);
    void set_frequency(CORBA::ULong channel, CORBA::Float f);
    void get_frequency(CORBA::ULong channel, CORBA::Float &f);

    void start(CORBA::ULong channel);
    void stop(CORBA::ULong channel);

    void set_values(const CF::Properties &values);

    void set_interpolation_rate(CORBA::ULong channel, CORBA::ULong I);
    void get_interpolation_range(CORBA::ULong channel, CORBA::ULong &imin, CORBA::ULong &imax, CORBA::ULong &istep);

 private:
    USRP_TX_Control_i();  // No default constructor
    USRP_TX_Control_i(const USRP_TX_Control_i &); // No copying

    USRP_i *usrp;

    float db_lo_freq;
    float db_lo_offset;
    bool lo_locked;
};

class TX_data_i : public POA_standardInterfaces::complexShort

{
  public:
    TX_data_i(USRP_i *_usrp);
    ~TX_data_i();

    void pushPacket(const PortTypes::ShortSequence &I, const PortTypes::ShortSequence &Q);

 private:
    USRP_i *usrp;

    short *tx_buf;
    unsigned int tx_buf_idx;
    int tx_buf_len;

    unsigned int tx_underruns;
};

// Definitions for uses ports

// Main USRP device definition
void rx_data_process(void *data);

class USRP_i : public virtual Device_impl

{
    friend class USRP_RX_Control_i;
    friend class USRP_TX_Control_i;

    friend void rx_data_process(void *);

  public:
    USRP_i(char *id, char *label, char *profile);

    static void do_rx_data_process(void *u) {((USRP_i *)u)->rx_data_process(); };
    static void do_tx_data_process(void *u) {((USRP_i *)u)->tx_data_process(); };

    // Methods from the SCA definition
    void start() throw (CF::Resource::StartError, CORBA::SystemException);
    void stop() throw (CF::Resource::StopError, CORBA::SystemException);
    CORBA::Object_ptr getPort(const char* portName) throw(CF::PortSupplier::UnknownPort, CORBA::SystemException);
    void initialize() throw (CF::LifeCycle::InitializeError, CORBA::SystemException);
    void configure(const CF::Properties &configProperties) throw (CORBA::SystemException, CF::PropertySet::InvalidConfiguration, CF::PropertySet::PartialConfiguration);
    void query(CF::Properties &configProperties) throw (CORBA::SystemException, CF::UnknownProperties);

    void releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException);

    omni_mutex rx_run;

    // These mutex's protect port and usrp config operations
    omni_mutex rx_control_access;
    omni_mutex tx_control_access;


 private:
    USRP_i();  // No default constructor
    USRP_i(const USRP_i&);  // No copying

    // RX data processing thread
    //    static void rx_data_process(void * data); ///\todo verify statis is the online way

    /// Prints warning about unsupported USRP daughter boards
    void PrintDaughterboardWarning(const char * db_name);

    // Port objects
    USRP_RX_Control_i* rx_control_port;
    USRP_TX_Control_i* tx_control_port;

    standardInterfaces_i::complexShort_u* rx_data_1_port;
    standardInterfaces_i::complexShort_u* rx_data_2_port;

    standardInterfaces_i::complexShort_p* tx_data_port;

    // usrp variables

    usrp_standard_rx *usrp_rx;
    usrp_standard_tx *usrp_tx;

    // Daughterboard data
    int rx_db0;
    int rx_db1;
    int tx_db0;
    int tx_db1;

    db_base *rx_db0_control;
    db_base *rx_db1_control;
    db_base *tx_db0_control;
    db_base *tx_db1_control;

    omni_thread *rx_thread;
    omni_thread *tx_thread;

    void rx_data_process();
    void tx_data_process();

    long set_rx_packet_count;
    long rx_packet_count;           ///< Number of packets to collect from USRP, -1 is forever
    unsigned int rx_packet_size;    ///< Number of samples to send to clients
    unsigned int rx_data_size;      ///< Size of words coming from USRP
    unsigned int number_of_channels;
    bool complex;                   ///< True for complex data from USRP
    unsigned int rx_overruns;

    bool rx_active;
    bool tx_active;

};
