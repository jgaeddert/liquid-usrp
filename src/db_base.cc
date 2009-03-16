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

#include <iostream>

#include "usrp_basic.h"
#include "usrp_standard.h"
#include "usrp_prims.h"

#include "db_base.h"

db_base::db_base(usrp_standard_rx *_u, unsigned int _which) : u(_u), urx(_u), utx(NULL), which(_which), tx(false)
{
    slot = which * 2 + 1;
}

db_base::db_base(usrp_standard_tx *_u, unsigned int _which) : u(_u), urx(NULL), utx(_u), which(_which), tx(true)
{
    slot = which * 2;
}

// destructor
db_base::~db_base()
{
}

int db_base::dbid()
{
    if (tx)
        return(utx->daughterboard_id(which));
    else
        return(urx->daughterboard_id(which));
}

std::string db_base::db_name()
{
    if (tx)
        return(usrp_dbid_to_string(utx->daughterboard_id(which)));
    else
        return(usrp_dbid_to_string(urx->daughterboard_id(which)));
}

void db_base::bypass_adc_buffers(bool bypass)
{
    if (tx)
        std::cerr << "Can't bypass adc buffers on TX daughter boards." << std::endl;
    else
        if (which == 0) {
            u->set_adc_buffer_bypass(0, bypass);
            u->set_adc_buffer_bypass(1, bypass);
        } else {
            u->set_adc_buffer_bypass(2, bypass);
            u->set_adc_buffer_bypass(3, bypass);
        }
}

void db_base::set_atr_mask(int mask)
{
    u->_write_fpga_reg(FR_ATR_MASK_0 + 3 * slot, mask);
}

void db_base::set_atr_txval(int txval)
{
    u->_write_fpga_reg(FR_ATR_TXVAL_0 + 3 * slot, txval);
}

void db_base::set_atr_rxval(int rxval)
{
    u->_write_fpga_reg(FR_ATR_RXVAL_0 + 3 * slot, rxval);
}

void db_base::get_pga_gain_range(float &gmin, float &gmax, float &gstep)
{
    if (urx) {
        gmin = urx->pga_min();
        gmax = urx->pga_max();
        gstep = urx->pga_db_per_step();
    } else if (utx) {
        gmin = utx->pga_min();
        gmax = utx->pga_max();
        gstep = utx->pga_db_per_step();
    } else
        ;  // This should never happen
}

void db_base::set_pga_gain(int which, float gain)
{
    if (urx)
        urx->set_pga(which, gain);
    else if (utx)
        utx->set_pga(which, gain);
    else
        ;  // This should never happen
}
