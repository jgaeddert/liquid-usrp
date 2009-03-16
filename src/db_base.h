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
#ifndef DB_BASE_H
#define DB_BASE_H

#include <string>

#include <usrp_standard.h>

class db_base {

public:
  db_base(usrp_standard_rx *u, unsigned int which);
  db_base(usrp_standard_tx *u, unsigned int which);
  virtual ~db_base();

  int dbid();
  std::string db_name();
  void bypass_adc_buffers(bool bypass);

  virtual bool set_db_freq(float lo_freq, float &actual_freq) = 0;
  virtual void set_gain(float db) = 0;
  // Add refclk code
  virtual void set_atr_mask(int mask);
  virtual void set_atr_txval(int txval);
  virtual void set_atr_rxval(int rxval);

  virtual void set_enable(bool on) { return; };
  virtual void set_auto_tr(bool on) { return; };
  virtual void select_rx_antenna(int antenna) {return; };

  virtual void get_freq_range(float &min, float &max, float &step) = 0;
  virtual void get_gain_range(float &min, float &max, float &step) = 0;

  void get_pga_gain_range(float &pga_min, float &pga_max, float &pga_step);
  void set_pga_gain(int which, float gain);

  virtual bool is_quadrature() = 0;
  virtual bool i_and_q_swapped() { return false; };
  virtual bool spectrum_inverted() { return false; };
  virtual bool db_has_lo() = 0;

protected:
  unsigned int slot;
  usrp_basic *u;
  usrp_standard_rx *urx;
  usrp_standard_tx *utx;
  unsigned int which;

private:
  bool tx;

  //Constants
  static const int FR_ATR_MASK_0 = 20;
  static const int FR_ATR_TXVAL_0 = 21;
  static const int FR_ATR_RXVAL_0 = 22;

};


#endif
