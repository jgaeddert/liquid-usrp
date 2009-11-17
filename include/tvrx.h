/****************************************************************************

Copyright 2005,2006 Virginia Polytechnic Institute and State University

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

#ifndef TVRX_H
#define TVRX_H

#include "usrp_spi_defs.h"

#include "db_base.h"

class tvrx : public db_base {

public:
  tvrx(usrp_standard_rx *urx, unsigned int which);
  virtual ~tvrx();

  bool set_db_freq(float lo_freq, float &actual_freq);
  void set_gain(float gain);
  
  void bw_range(float &min, float &max);
  void get_freq_range(float &min, float &max, float &step);
  void get_gain_range(float &min, float &max, float &step);

  bool is_quadrature() { return false; };
  bool db_has_lo() { return true; };

protected:
  void set_inverted(bool inverted_in);
  void set_first_IF_freq(float first_IF_freq_in);
  void set_second_IF_freq(float second_IF_freq_in);

  bool spectrum_inverted() { return INVERTED; };
  float first_if_freq() { return FIRST_IF_FREQ; };
  float second_if_freq() { return SECOND_IF_FREQ; };
  
private:
//same for all revs
int i2c_addr;
int reference_divisor;
bool fast_tuning;

//differs for rev1 vs rev2, rev3
float FIRST_IF_FREQ;
float SECOND_IF_FREQ;
bool INVERTED;



//Helper functions
  char control_byte_1(bool fast_tuning_in, unsigned int reference_divisor_in);
  char control_byte_2(float target_freq, bool shutdown_tx_pga);

//Gain helper functions
  void set_rfagc(unsigned int rf_gain_in);
  void set_ifagc(unsigned int if_gain_in);
  void set_pga(unsigned int pga_in);
  
};

class tvrx_rev2 : public tvrx {

public:
  tvrx_rev2(usrp_standard_rx *urx, unsigned int which);

};

class tvrx_rev3 : public tvrx {

public:
  tvrx_rev3(usrp_standard_rx *urx, unsigned int which);

};

#endif

