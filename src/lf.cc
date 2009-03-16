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

#include <string>
#include <iostream>

#include <math.h>
#include <unistd.h>

#include "db_base.h"
#include "lf.h"


db_lf::db_lf(usrp_standard_rx *u, unsigned int _which) : db_base(u, _which)
{

}

db_lf::db_lf(usrp_standard_tx *u, unsigned int _which) : db_base(u, _which)
{

}

db_lf_rx::db_lf_rx(usrp_standard_rx *_urx, unsigned int _w) : db_lf(_urx, _w)
{
    u = _urx;

    bypass_adc_buffers(true);

    float g_min, g_max, g_step;
    get_gain_range(g_min, g_max, g_step);
    set_gain((g_min + g_max) / 2.0);
}

db_lf_tx::db_lf_tx(usrp_standard_tx *_utx, unsigned int _w) : db_lf(_utx, _w)
{
    u = _utx;

    u->_write_fpga_reg(40 + slot, 0);  // Disable reference clock

    float g_min, g_max, g_step;
    get_gain_range(g_min, g_max, g_step);
    set_gain((g_min + g_max) / 2.0);
}

db_lf::~db_lf()
{

}

void db_lf::get_freq_range(float &min, float &max, float &step)
{
    min = 0;
    max = 32e6;
    step = 1e-6;
}

void db_lf::get_gain_range(float &gmin, float &gmax, float &gstep)
{
    get_pga_gain_range(gmin, gmax, gstep);        
}

bool db_lf::set_db_freq(float freq_in, float &actual_freq)
{
    actual_freq = 0;
    return true;
}

void db_lf::set_gain(float gain_in)
{
    float p_min, p_max, p_step;
    get_pga_gain_range(p_min, p_max, p_step);

    if ((gain_in < p_min) || (gain_in > p_max)) {
        return;
    }

    if (urx) {
        if (which == 0) {
            urx->set_pga(0, gain_in);
            urx->set_pga(1, gain_in);
        } else {
            urx->set_pga(2, gain_in);
            urx->set_pga(3, gain_in);
        }
    } else if (utx) {
        if (which == 0) {
            utx->set_pga(0, gain_in);
            utx->set_pga(1, gain_in);
        } else {
            utx->set_pga(2, gain_in);
            utx->set_pga(3, gain_in);
        }
    }
}



