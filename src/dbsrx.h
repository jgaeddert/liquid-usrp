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

#ifndef DBSRX_H
#define DBSRX_H

#include "usrp_spi_defs.h"

#include "db_base.h"

class dbsrx : public db_base {

public:
  dbsrx(usrp_standard_rx *urx, unsigned int which);
  virtual ~dbsrx();
  
  bool set_db_freq(float lo_freq, float &actual_freq);
  void set_gain(float gain);
  void set_bw(float bw);

  void bw_range(float &min, float &max);
  void get_freq_range(float &min, float &max, float &step);
  void get_gain_range(float &min, float &max, float &step);

  bool is_quadrature() {return true;}; 
  bool db_has_lo() {return true;}; //OSSIE specific? ->Yup!

private:
  dbsrx();
  int i2c_addr;
  int ref_clkreg;
  int n;
  int div2;
  int osc;
  int cp;
  int r;
  int r_int;
  int fdac;
  int m;
  int dl;
  int ade;
  int adl;
  int gc1;
  int gc2;
  int diag;
  float lo_offset;
  static const int REFCLK_DIVISOR = 16; //0x10
  static const float MASTER_FPGA_FREQ = 64E6;
  static const int CLOCK_OUT = 1; //Clock is on lowest bit
  static const int REFCLK_ENABLE = 0x80;
  static const int REFCLK_DIVISOR_MASK = 0x7f;



//Helper functions
  void write_reg(int regno, int value);
  //void write_regs(int start_regno, int *values);
  void read_status(int &status_byte1, int &status_byte2);
  void send_reg(int regno);

//BW setting helper functions
  void set_m(int m_in);
  void set_fdac(int fdac_in);

//Gain setting helper functions
  void set_dl(int dl_in);
  void set_gc2(int gc2_in);
  void set_gc1(int gc1_in);
  void set_pga(int pga_gain_in);

//Freq setting helper functions
  void set_osc(int osc_in);
  void set_cp(int cp_in);
  void set_n(int n_in);
  void set_div2(int div2_in);
  void set_r(int r_in);
  void set_ade(int ade_in);
  int refclk_divisor() {return REFCLK_DIVISOR;};
  float refclk_freq() {return MASTER_FPGA_FREQ/REFCLK_DIVISOR;};

};

#endif
