/****************************************************************************

Copyright 2007 Virginia Polytechnic Institute and State University

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

#ifndef LF_H
#define LF_H

#include "usrp_spi_defs.h"
#include "db_base.h"

class db_lf : public db_base {

public:
  db_lf(usrp_standard_rx *u, unsigned int which);
  db_lf(usrp_standard_tx *u, unsigned int which);

  virtual ~db_lf();

  void get_freq_range(float &min, float &max, float &step);
  void get_gain_range(float &min, float &max, float &step);

  bool set_db_freq(float freq_in, float &actual_freq);
  void set_gain(float gain_in);

  bool is_quadrature() { return false; };
  bool db_has_lo() { return false; };

private:
  db_lf();

};

class db_lf_rx : public db_lf {

public:
  db_lf_rx(usrp_standard_rx *u, unsigned int which);

  bool i_and_q_swapped() { return false; };
private:
};

class db_lf_tx : public db_lf {

public:
  db_lf_tx(usrp_standard_tx *u, unsigned int which);

  bool i_and_q_swapped() { return false; };
private:
};

#endif
