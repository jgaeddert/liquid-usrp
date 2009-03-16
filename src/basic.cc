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

#include <string>
#include <iostream>

#include <math.h>
#include <unistd.h>

#include "db_base.h"
#include "basic.h"


db_basic::db_basic(usrp_standard_rx *u, unsigned int _which) : db_base(u, _which)
{

}

db_basic::db_basic(usrp_standard_tx *u, unsigned int _which) : db_base(u, _which)
{

}

db_basic_rx::db_basic_rx(usrp_standard_rx *_urx, unsigned int _w) : db_basic(_urx, _w)
{

    bypass_adc_buffers(true);

    u = _urx;

    float gmin, gmax, gstep;
    get_gain_range(gmin, gmax, gstep);
    set_gain((gmin+gmax)/2.0);
}

db_basic_tx::db_basic_tx(usrp_standard_tx *_utx, unsigned int _w) : db_basic(_utx, _w)
{

    u = _utx;

    u->_write_fpga_reg(40 + slot, 0);  // Disable ref clk

    float gmin, gmax, gstep;
    get_gain_range(gmin, gmax, gstep);
    set_gain((gmin+gmax)/2.0);
}

db_basic::~db_basic()
{

}

void db_basic::set_gain(float gain)
{
    float pmin, pmax, pstep;
    get_pga_gain_range(pmin, pmax, pstep);

    if ((gain < pmin) || (gain > pmax))
        return; /// \todo throw exception

    if (utx) {
        if (which == 0) {
            utx->set_pga(0, gain);
            utx->set_pga(1, gain);
        } else {
            utx->set_pga(2, gain);
            utx->set_pga(3, gain);
        }
    } else if (urx) {
        if (which == 0) {
            urx->set_pga(0, gain);
            urx->set_pga(1, gain);
        } else {
            urx->set_pga(2, gain);
            urx->set_pga(3, gain);
        }
    }
}

void db_basic::get_freq_range(float &min, float &max, float &step)
{
    min = 0;
    max = 32e6;
    step = 1e-6;
}

void db_basic::get_gain_range(float &gmin, float &gmax, float &gstep)
{

    get_pga_gain_range(gmin, gmax, gstep);
        
}

bool db_basic::set_db_freq(float freq, float &actual_freq)
{
    /// \todo throw exception indicating no db lo to set
    return false;
}

