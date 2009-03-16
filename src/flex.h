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

#ifndef FLEX_H
#define FLEX_H

#include "usrp_spi_defs.h"

#include "db_base.h"

class flex_vco_control {

public:
  flex_vco_control(int R_DIV, int P, int CP2, int CP1, int DIVSEL, int DIV2, int CPGAIN, int freq_mult);

  virtual ~flex_vco_control();

  void compute_regs(float frequency, int &R, int &control, int &N, float &actual_frequency); 
  virtual void freq_range(float &lo, float &hi, float &step) = 0;

  void setDIV2(unsigned int _DIVSEL);
  void setREFCLK_DIVISOR(unsigned int RCLK_D);
  void setR_DIV(unsigned int REF_D);

private:
  int refclk_divisor() { return REFCLK_DIVISOR; } ;
  int prescaler();

  // Common frequency setting variables
  // R register common values
  static const int R_RSV = 0;
  static const int BSC = 3;
  static const int TEST = 0;
  static const int LDP = 1;
  static const int ABP = 0;

  // N register common values
  static const int N_RSV = 0;

  // Control register common values
  static const int PD = 0;
  static const int PL = 0;
  static const int MTLD = 1;
  static const int CPG = 0;
  static const int CP3S = 0;
  static const int PDP = 1;
  static const int MUXOUT = 1;
  static const int CR = 0;
  static const int PC = 1;

  // Band specific values
  // R register
  int R_DIV; //16 is Default value for non-mimo and mimo_b versions

  // C register
  int P;
  int CP2;
  int CP1;

  // N register
  int DIVSEL;
  int DIV2;
  int CPGAIN;

  int freq_mult;
  int REFCLK_DIVISOR; //1 is Default value for non-mimo and mimo_b versions

};

class flex400_vco_control : public flex_vco_control {

public:
  flex400_vco_control();

  void freq_range(float &lo, float &hi, float &step) { lo = 400e6; hi = 500e6; step = 1e6; } ;

 
};

class flex900_vco_control : public flex_vco_control {

public:
  flex900_vco_control();

  void freq_range(float &lo, float &hi, float &step) { lo = 800e6; hi = 1000e6; step = 4e6; } ;

 
};

class flex1200_vco_control : public flex_vco_control {

public:
  flex1200_vco_control();

  void freq_range(float &lo, float &hi, float&step) { lo = 1150e6; hi = 1350e6; step = 4e6; } ;


};

class flex1800_vco_control : public flex_vco_control {

public:
  flex1800_vco_control();

  void freq_range(float &lo, float &hi, float &step) { lo = 1600e6; hi = 2000e6; step = 4e6; };


};

class flex2400_vco_control : public flex_vco_control {

public:
  flex2400_vco_control();

  void freq_range(float &lo, float &hi, float &step) { lo = 2300e6; hi = 2500e6; step = 4e6; } ;

};

class db_flex : public db_base {

public:
  db_flex(usrp_standard_rx *u, unsigned int which, flex_vco_control *vco);
  db_flex(usrp_standard_tx *u, unsigned int which, flex_vco_control *vco);
  virtual ~db_flex();

  bool set_db_freq(float lo_freq, float &actual_freq);
  void set_gain(float gain);

  bool is_quadrature() { return true; };

  void get_freq_range(float &min, float &max, float &step);

  void select_rx_antenna(int antenna);

  void set_auto_tr(bool on);
  void set_enable(bool on);

  bool lock_detect();
  bool db_has_lo() {return true;};

protected:
  // Daughterboard IO pin defs common to all flex series
  static const int AUX_RXAGC = (1<<8);
  static const int POWER_UP = (1<<7); // Enables daughterboard power supply
  static const int RX_TXN = (1<<6); // TX only: T/R antenna switch for TX/RX port
  static const int RX2_RX1N = (1<<6); // RX only: antenna switch between RX@ and TX/RX port
  static const int ENABLE = (1<<5); // Enables mixer
  static const int AUX_SEN = (1<<4);
  static const int AUX_SCLK = (1<<3);
  static const int PLL_LOCK_DETECT = (1<<2);
  static const int AUX_SDO = (1<<1);
  static const int CLOCK_OUT = (1<<0);

  int power_on;  // Deal with odd daughter board power on/off behavior
  int power_off;
  
private:
  db_flex();

  void write_all(int R, int control, int N);
  void write_control(int control);
  void write_R(int R);
  void write_N(int N);
  void write_it(int N);
  

  flex_vco_control *vco;
  bool first; // Set until after first write to vco

  int spi_format;
  int spi_enable;
};

class db_flex400_rx : public db_flex {

public:
  db_flex400_rx(usrp_standard_rx *u, unsigned int which);

  bool i_and_q_swapped() { return true; };
  void get_gain_range(float &min, float &max, float &step);

protected:
  class flex400_vco_control vco400;

};

class db_flex400_rx_mimo_a : public db_flex400_rx {

public:
  db_flex400_rx_mimo_a(usrp_standard_rx *u, unsigned int which);

};

class db_flex400_rx_mimo_b : public db_flex400_rx {

public:
  db_flex400_rx_mimo_b(usrp_standard_rx *u, unsigned int which);

};

class db_flex900_rx : public db_flex {

public:
  db_flex900_rx(usrp_standard_rx *u, unsigned int which);

  bool i_and_q_swapped() { return true; };
  void get_gain_range(float &min, float &max, float &step);

protected:
  class flex900_vco_control vco900;

};

class db_flex900_rx_mimo_a : public db_flex900_rx {

public:
  db_flex900_rx_mimo_a(usrp_standard_rx *u, unsigned int which);

};

class db_flex900_rx_mimo_b : public db_flex900_rx {

public:
  db_flex900_rx_mimo_b(usrp_standard_rx *u, unsigned int which);

};

class db_flex1200_rx : public db_flex {

public:
  db_flex1200_rx(usrp_standard_rx *u, unsigned int which);

  bool i_and_q_swapped() { return true; };
  void get_gain_range(float &min, float &max, float &step);

protected:
  class flex1200_vco_control vco1200;

};

class db_flex1200_rx_mimo_a : public db_flex1200_rx {

public:
  db_flex1200_rx_mimo_a(usrp_standard_rx *u, unsigned int which);

};

class db_flex1200_rx_mimo_b : public db_flex1200_rx {

public:
  db_flex1200_rx_mimo_b(usrp_standard_rx *u, unsigned int which);

};

class db_flex1800_rx : public db_flex {

public:
  db_flex1800_rx(usrp_standard_rx *u, unsigned int which);

  bool i_and_q_swapped() { return true; };
  void get_gain_range(float &min, float &max, float &step);

protected:
  class flex1800_vco_control vco1800;

};

class db_flex1800_rx_mimo_a : public db_flex1800_rx {

public:
  db_flex1800_rx_mimo_a(usrp_standard_rx *u, unsigned int which);

};

class db_flex1800_rx_mimo_b : public db_flex1800_rx {

public:
  db_flex1800_rx_mimo_b(usrp_standard_rx *u, unsigned int which);

};

class db_flex2400_rx : public db_flex {

public:
  db_flex2400_rx(usrp_standard_rx *u, unsigned int which);

  bool i_and_q_swapped() { return true; };
  void get_gain_range(float &min, float &max, float &step);

protected:
  class flex2400_vco_control vco2400;

};

class db_flex2400_rx_mimo_a : public db_flex2400_rx {

public:
  db_flex2400_rx_mimo_a(usrp_standard_rx *u, unsigned int which);

};

class db_flex2400_rx_mimo_b : public db_flex2400_rx {

public:
  db_flex2400_rx_mimo_b(usrp_standard_rx *u, unsigned int which);

};

class db_flex400_tx : public db_flex {

public:
  db_flex400_tx(usrp_standard_tx *u, unsigned int which);

  void get_gain_range(float &min, float &max, float &step);

protected:
  class flex400_vco_control vco400;

};

class db_flex400_tx_mimo_a : public db_flex400_tx {

public:
  db_flex400_tx_mimo_a(usrp_standard_tx *u, unsigned int which);

};

class db_flex400_tx_mimo_b : public db_flex400_tx {

public:
  db_flex400_tx_mimo_b(usrp_standard_tx *u, unsigned int which);

};

class db_flex900_tx : public db_flex {

public:
  db_flex900_tx(usrp_standard_tx *u, unsigned int which);

  void get_gain_range(float &min, float &max, float &step);

protected:
  class flex900_vco_control vco900;

};

class db_flex900_tx_mimo_a : public db_flex900_tx {

public:
  db_flex900_tx_mimo_a(usrp_standard_tx *u, unsigned int which);

};

class db_flex900_tx_mimo_b : public db_flex900_tx {

public:
  db_flex900_tx_mimo_b(usrp_standard_tx *u, unsigned int which);

};

class db_flex1200_tx : public db_flex {

public:
  db_flex1200_tx(usrp_standard_tx *u, unsigned int which);

  void get_gain_range(float &min, float &max, float &step);

protected:
  class flex1200_vco_control vco1200;

};

class db_flex1200_tx_mimo_a : public db_flex1200_tx {

public:
  db_flex1200_tx_mimo_a(usrp_standard_tx *u, unsigned int which);

};

class db_flex1200_tx_mimo_b : public db_flex1200_tx {

public:
  db_flex1200_tx_mimo_b(usrp_standard_tx *u, unsigned int which);

};

class db_flex1800_tx : public db_flex {

public:
  db_flex1800_tx(usrp_standard_tx *u, unsigned int which);

  void get_gain_range(float &min, float &max, float &step);

protected:
  class flex1800_vco_control vco1800;

};

class db_flex1800_tx_mimo_a : public db_flex1800_tx {

public:
  db_flex1800_tx_mimo_a(usrp_standard_tx *u, unsigned int which);

};

class db_flex1800_tx_mimo_b : public db_flex1800_tx {

public:
  db_flex1800_tx_mimo_b(usrp_standard_tx *u, unsigned int which);

};

class db_flex2400_tx : public db_flex {

public:
  db_flex2400_tx(usrp_standard_tx *u, unsigned int which);

  void get_gain_range(float &min, float &max, float &step);

protected:
  class flex2400_vco_control vco2400;

};

class db_flex2400_tx_mimo_a : public db_flex2400_tx {

public:
  db_flex2400_tx_mimo_a(usrp_standard_tx *u, unsigned int which);

};

class db_flex2400_tx_mimo_b : public db_flex2400_tx {

public:
  db_flex2400_tx_mimo_b(usrp_standard_tx *u, unsigned int which);

};

#endif
