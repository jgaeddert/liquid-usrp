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

#include "ossie/debug.h"

#include "db_base.h"
#include "flex.h"

flex_vco_control::flex_vco_control(int _R_DIV, int _P, int _CP2, int _CP1, int _DIVSEL, int _DIV2, int _CPGAIN, int _freq_mult) : R_DIV(_R_DIV), P(_P), CP2(_CP2), CP1(_CP1), DIVSEL(_DIVSEL), DIV2(_DIV2), CPGAIN(_CPGAIN), freq_mult(_freq_mult), REFCLK_DIVISOR(1)
{

}

// destructor
flex_vco_control::~flex_vco_control()
{
}

void flex_vco_control::compute_regs(float frequency, int &R, int &control, int &N, float &actual_frequency)
{
    float refclk_freq = 64.0e6 / refclk_divisor();
    
    float phdet_freq = refclk_freq / R_DIV;
    float desired_n = round(frequency * freq_mult / phdet_freq);
    actual_frequency = desired_n * phdet_freq;
    int B_DIV = (int) floor(desired_n / prescaler());
    int A_DIV = (int) floor(desired_n - prescaler() * B_DIV);
    DEBUG(3, USRP, "phdet_freq = " << phdet_freq << "  desired_n = " << desired_n << " actual_frequency = " << actual_frequency << "  B_DIV = " << B_DIV << "  A_DIV = " << A_DIV)

    if (B_DIV < A_DIV)
        return ;

    R = 0;
    control = 0;
    N = 0;

    R = (R_RSV<<22) | (BSC<<20) | (TEST<<19) | (LDP<<18) | (ABP<<16) | (R_DIV<<2);

    control = (P<<22) | (PD<<20) | (CP2<<17) | (CP1<<14) | (PL<<12) | (MTLD<<11) | (CPG<<10) | (CP3S<<9) | (PDP<<8) | (MUXOUT<<5) | (CR<<4) | (PC<<2);

    N = (DIVSEL<<23) | (DIV2<<22) | (CPGAIN<<21) | (B_DIV<<8) | (N_RSV<<7) | (A_DIV<<2);

    actual_frequency /= freq_mult;
}

int flex_vco_control::prescaler()
{
    if (P == 0)
        return 8;
    else if (P == 1)
        return 16;
    else
        return 32;
}

flex400_vco_control::flex400_vco_control() : flex_vco_control(16, 0, 7, 7, 0, 1, 0, 2)
{

}

flex900_vco_control::flex900_vco_control() : flex_vco_control(16, 1, 7, 7, 0, 1, 0, 2)
{

}

flex1200_vco_control::flex1200_vco_control() : flex_vco_control(16, 1, 7, 7, 0, 1, 0, 2)
{

}

flex1800_vco_control::flex1800_vco_control() : flex_vco_control(16, 1, 7, 7, 0, 0, 0, 1)
{

}

flex2400_vco_control::flex2400_vco_control() : flex_vco_control(16, 1, 7, 7, 0, 0, 0, 1)
{

}

void flex_vco_control::setDIV2(unsigned int D)
{
    DIV2 = D;
}

void flex_vco_control::setREFCLK_DIVISOR(unsigned int RCLK_D)
{
    REFCLK_DIVISOR = RCLK_D;
}

void flex_vco_control::setR_DIV(unsigned int REF_D)
{
    R_DIV = REF_D;
}

db_flex::db_flex(usrp_standard_rx *_urx, unsigned int _w, flex_vco_control *_vco) : db_base(_urx, _w), vco(_vco), first(true), spi_format(SPI_FMT_MSB | SPI_FMT_HDR_0)
{

    if (which == 0)
        spi_enable = SPI_ENABLE_RX_A;
    else
        spi_enable = SPI_ENABLE_RX_B;

    urx->_write_oe(which, (POWER_UP|RX2_RX1N|ENABLE), 0xffff);
    urx->write_io(which, (power_on|RX2_RX1N|ENABLE), (POWER_UP|RX2_RX1N|ENABLE));
    select_rx_antenna(0);
    
    bypass_adc_buffers(true);

    set_auto_tr(false);

    u = _urx;

}

db_flex::db_flex(usrp_standard_tx *_utx, unsigned int _w, flex_vco_control *_vco) : db_base(_utx, _w), vco(_vco), first(true), spi_format(SPI_FMT_MSB | SPI_FMT_HDR_0)
{

    if (which == 0)
        spi_enable = SPI_ENABLE_TX_A;
    else
        spi_enable = SPI_ENABLE_TX_B;

    u = _utx;

    u->_write_fpga_reg(40 + slot, 0);  // Disable ref clk
}

db_flex::~db_flex()
{

    if (urx)
        urx->write_io(which, power_off, (POWER_UP|ENABLE));
    else if (utx) {
        utx->write_io(which, (power_off|RX_TXN), (POWER_UP|RX_TXN));
    }

    set_auto_tr(false);
}

bool db_flex::set_db_freq(float lo_freq, float &actual_freq)
{
    float fmin, fmax, fstep;
    vco->freq_range(fmin, fmax, fstep);
    int R, control, N;

    vco->compute_regs(lo_freq, R, control, N, actual_freq);

    write_all(R, control, N);

    return(lock_detect());
}

bool db_flex::lock_detect()
{
    usleep(1000);

    if (urx) {
        
        if (urx->read_io(which) & PLL_LOCK_DETECT)
            return true;
        else if (urx->read_io(which) & PLL_LOCK_DETECT)
            return true;
        else
            return false;
    } else if (utx) {
        if (utx->read_io(which) & PLL_LOCK_DETECT)
            return true;
        else if (utx->read_io(which) & PLL_LOCK_DETECT)
            return true;
        else
            return false;
    } else 
        return false; // This should never happen        
}

void db_flex::write_all(int R, int control, int N)
{
    write_R(R);
    write_control(control);
    if (first) {
        usleep(10000); // sleep for 10mS on first write to vco
        first = false;
    }
    write_N(N);
}

void db_flex::write_control(int val)
{
    write_it((val & ~0x3) | 0);
}

void db_flex::write_R(int val)
{
    write_it((val & ~0x3) | 1);
}

void db_flex::write_N(int val)
{
    write_it((val & ~0x3) | 2);
}

void db_flex::write_it(int val)
{
    std::string s = "   ";

    s[0] = (val>>16) & 0xff;
    s[1] = (val>>8) & 0xff;
    s[2] = (val) & 0xff;
    u->_write_spi(0, spi_enable, spi_format, s);
}

void db_flex::set_gain(float gain)
{
    float gmin, gmax, gstep;
    get_gain_range(gmin, gmax, gstep);

    float pmin, pmax, pstep;
    get_pga_gain_range(pmin, pmax, pstep);

    if (utx) {
        if (which == 0) {
            utx->set_pga(0, gmax);
            utx->set_pga(1, gmax);
        } else {
            utx->set_pga(2, gmax);
            utx->set_pga(3, gmax);
        }
    } else if (urx) {
        float pga_gain, agc_gain;
        float maxgain = gmax - pmax;
        if (gain > maxgain) {
            pga_gain = gain - maxgain;
            if (pga_gain > pmax) {
                pga_gain = pmax;
            }
            agc_gain = maxgain;
        } else {
            pga_gain = 0;
            agc_gain = gain;
        }

        float V_maxgain = 0.2;
        float V_mingain = 1.2;
        float V_fullscale = 3.3;
        float dac_value = (agc_gain * (V_maxgain - V_mingain) / maxgain + V_mingain) * 4096 / V_fullscale;
        if (dac_value < 0)
            dac_value = 0;
        else if (dac_value > 4096)
            dac_value = 4096;

        urx->write_aux_dac(which, 0, (int) dac_value);
        if (which == 0) {
            urx->set_pga(0, pga_gain);
            urx->set_pga(1, pga_gain);
        } else {
            urx->set_pga(2, pga_gain);
            urx->set_pga(3, pga_gain);
        }
    }
        

}

void db_flex::get_freq_range(float &min, float &max, float &step)
{
    vco->freq_range(min, max, step);
}

void db_flex::select_rx_antenna(int antenna)
{
    if (urx) { // Can only switch rx antenna if we are an rx db
        if (antenna == 0) {  // Select TX/RX port
            urx->write_io(which, 0, RX2_RX1N);
        } else if (antenna == 1) {
            urx->write_io(which, RX2_RX1N, RX2_RX1N);
        } else
            ; // Error
    }
}

void db_flex::set_auto_tr(bool on)
{
    if (urx) {
        if (on) {
            set_atr_mask(ENABLE);
            set_atr_txval(0);
            set_atr_rxval(ENABLE);
        } else {
            set_atr_mask(0);
            set_atr_txval(0);
            set_atr_rxval(0);
        }
    } else if (utx) {
        if (on) {
            set_atr_mask(RX_TXN|ENABLE);
            set_atr_txval(0 | ENABLE);
            set_atr_rxval(RX_TXN | 0);
        } else {
            set_atr_mask(0);
            set_atr_txval(0);
            set_atr_rxval(0);
        }
    }
}

void db_flex::set_enable(bool on)
{
    if (utx) { // You can only turn on the transmitter
        int v;
        int mask = (RX_TXN | ENABLE);
        if (on)
            v = ENABLE;
        else
            v = RX_TXN;
        utx->write_io(which, v, mask);
    }
}

db_flex400_rx::db_flex400_rx(usrp_standard_rx *_u, unsigned int _w) : db_flex(_u, _w, &vco400)
{
    vco400.setDIV2(0);
    power_on = POWER_UP;
    power_off = ~POWER_UP;

    urx->_write_oe(which, 0, 0xffff);
    urx->_write_oe(which, (POWER_UP|RX2_RX1N|ENABLE), 0xffff);
    urx->write_io(which, (power_on|RX2_RX1N|ENABLE), (POWER_UP|RX2_RX1N|ENABLE));

    set_auto_tr(false);

    float gmax, gmin, gstep;
    get_gain_range(gmin, gmax, gstep);
    DEBUG(3, USRP, "Flex 400 Gain min = " << gmin << " max = " << gmax << " Initial setting = " << (gmin + gmax)/2.0)
    set_gain((gmin + gmax) / 2.0);

} 

void db_flex400_rx::get_gain_range(float &gmin, float &gmax, float &gstep)
{
    get_pga_gain_range(gmin, gmax, gstep);
    gstep = 0.035;
    gmax += 45;
}

db_flex400_tx::db_flex400_tx(usrp_standard_tx *_u, unsigned int _w) : db_flex(_u, _w, &vco400)
{
    vco400.setDIV2(1);
    power_on = POWER_UP;
    power_off = ~POWER_UP;

    utx->_write_oe(which, 0, 0xffff);
    utx->_write_oe(which, (POWER_UP|RX_TXN|ENABLE), 0xffff);
    utx->write_io(which, (power_on|RX_TXN), (POWER_UP|RX_TXN|ENABLE));

    set_auto_tr(false);


    float gmax, gmin, gstep;
    get_gain_range(gmin, gmax, gstep);
    DEBUG(3, USRP, "In db_flex TX constructor which = " << which << " gain set to = " << gmax)
    set_gain(gmax);

} 

void db_flex400_tx::get_gain_range(float &gmin, float &gmax, float &gstep)
{
   get_pga_gain_range(gmin, gmax, gstep);
   gmin = gmax;
   gstep = 1.0;
}

db_flex400_rx_mimo_a::db_flex400_rx_mimo_a(usrp_standard_rx *_u, unsigned int _w) : db_flex400_rx(_u, _w)
{
    vco400.setR_DIV(1);
    vco400.setREFCLK_DIVISOR(16);         
} 

db_flex400_tx_mimo_a::db_flex400_tx_mimo_a(usrp_standard_tx *_u, unsigned int _w) : db_flex400_tx(_u, _w)
{
    vco400.setR_DIV(1);
    vco400.setREFCLK_DIVISOR(16);
}

db_flex400_rx_mimo_b::db_flex400_rx_mimo_b(usrp_standard_rx *_u, unsigned int _w) : db_flex400_rx(_u, _w)
{
         
} 

db_flex400_tx_mimo_b::db_flex400_tx_mimo_b(usrp_standard_tx *_u, unsigned int _w) : db_flex400_tx(_u, _w)
{

}

db_flex900_rx::db_flex900_rx(usrp_standard_rx *_u, unsigned int _w) : db_flex(_u, _w, &vco900)
{   
    power_on = ~POWER_UP;
    power_off = ~POWER_UP;  // Powering it off kills the serial bus
    std::cout << "running db_flex_900 constructor" << std::endl;
    urx->_write_oe(which, 0, 0xffff);
    urx->_write_oe(which, (POWER_UP|RX2_RX1N|ENABLE), 0xffff);
    urx->write_io(which, (power_on|RX2_RX1N|ENABLE), (POWER_UP|RX2_RX1N|ENABLE));

    set_auto_tr(false);

    float gmax, gmin, gstep;
    get_gain_range(gmin, gmax, gstep);
    set_gain((gmin + gmax) / 2.0);

} 

void db_flex900_rx::get_gain_range(float &gmin, float &gmax, float &gstep)
{
    get_pga_gain_range(gmin, gmax, gstep);
    gstep = 0.05;
    gmax += 70;
}

db_flex900_tx::db_flex900_tx(usrp_standard_tx *_u, unsigned int _w) : db_flex(_u, _w, &vco900)
{   
    power_on = ~POWER_UP;
    power_off = ~POWER_UP;  // Powering it off kills the serial bus

    std::cout << "In db_flex TX constructor which = " << which << " power on = " << power_on << std::endl;
    utx->_write_oe(which, 0, 0xffff);
    utx->_write_oe(which, (POWER_UP|RX_TXN|ENABLE), 0xffff);
    utx->write_io(which, (power_on|RX_TXN), (POWER_UP|RX_TXN|ENABLE));

    set_auto_tr(false);
    float gmax, gmin, gstep;
    get_gain_range(gmin, gmax, gstep);
    set_gain(gmax);

} 

void db_flex900_tx::get_gain_range(float &gmin, float &gmax, float &gstep)
{
   get_pga_gain_range(gmin, gmax, gstep);
   gmin = gmax;
   gstep = 1.0;
}

db_flex900_rx_mimo_a::db_flex900_rx_mimo_a(usrp_standard_rx *_u, unsigned int _w) : db_flex900_rx(_u, _w)
{
    vco900.setR_DIV(1);
    vco900.setREFCLK_DIVISOR(16);         
} 

db_flex900_tx_mimo_a::db_flex900_tx_mimo_a(usrp_standard_tx *_u, unsigned int _w) : db_flex900_tx(_u, _w)
{
    vco900.setR_DIV(1);
    vco900.setREFCLK_DIVISOR(16);
}

db_flex900_rx_mimo_b::db_flex900_rx_mimo_b(usrp_standard_rx *_u, unsigned int _w) : db_flex900_rx(_u, _w)
{
         
} 

db_flex900_tx_mimo_b::db_flex900_tx_mimo_b(usrp_standard_tx *_u, unsigned int _w) : db_flex900_tx(_u, _w)
{

}

db_flex1200_rx::db_flex1200_rx(usrp_standard_rx *_u, unsigned int _w) : db_flex(_u, _w, &vco1200)
{
    power_on = ~POWER_UP;
    power_off = ~POWER_UP;  // Powering it off kills the serial bus

    urx->_write_oe(which, 0, 0xffff);
    urx->_write_oe(which, (POWER_UP|RX2_RX1N|ENABLE), 0xffff);
    urx->write_io(which, (power_on|RX2_RX1N|ENABLE), (POWER_UP|RX2_RX1N|ENABLE));

    set_auto_tr(false);

    float gmax, gmin, gstep;
    get_gain_range(gmin, gmax, gstep);
    set_gain((gmin + gmax) / 2.0);

} 

void db_flex1200_rx::get_gain_range(float &gmin, float &gmax, float &gstep)
{
    get_pga_gain_range(gmin, gmax, gstep);
    gstep = 0.05;
    gmax += 70;
}

db_flex1200_tx::db_flex1200_tx(usrp_standard_tx *_u, unsigned int _w) : db_flex(_u, _w, &vco1200)
{
    power_on = ~POWER_UP;
    power_off = ~POWER_UP;  // Powering it off kills the serial bus

    std::cout << "In db_flex TX constructor which = " << which << " power on = " << power_on << std::endl;
    utx->_write_oe(which, 0, 0xffff);
    utx->_write_oe(which, (POWER_UP|RX_TXN|ENABLE), 0xffff);
    utx->write_io(which, (power_on|RX_TXN), (POWER_UP|RX_TXN|ENABLE));

    set_auto_tr(false);
    float gmax, gmin, gstep;
    get_gain_range(gmin, gmax, gstep);
    set_gain(gmax);

} 

void db_flex1200_tx::get_gain_range(float &gmin, float &gmax, float &gstep)
{
   get_pga_gain_range(gmin, gmax, gstep);
   gmin = gmax;
   gstep = 1.0;
}

db_flex1200_rx_mimo_a::db_flex1200_rx_mimo_a(usrp_standard_rx *_u, unsigned int _w) : db_flex1200_rx(_u, _w)
{
    vco1200.setR_DIV(1);
    vco1200.setREFCLK_DIVISOR(16);         
} 

db_flex1200_tx_mimo_a::db_flex1200_tx_mimo_a(usrp_standard_tx *_u, unsigned int _w) : db_flex1200_tx(_u, _w)
{
    vco1200.setR_DIV(1);
    vco1200.setREFCLK_DIVISOR(16);
}

db_flex1200_rx_mimo_b::db_flex1200_rx_mimo_b(usrp_standard_rx *_u, unsigned int _w) : db_flex1200_rx(_u, _w)
{
         
} 

db_flex1200_tx_mimo_b::db_flex1200_tx_mimo_b(usrp_standard_tx *_u, unsigned int _w) : db_flex1200_tx(_u, _w)
{

}

/*
db_flex1800_rx::db_flex1800_rx(usrp_standard_rx *_u, unsigned int _w) : db_flex(_u, _w, &vco1800)
{
    power_on = ~POWER_UP;
    power_off = ~POWER_UP;  // Powering it off kills the serial bus

    urx->_write_oe(which, 0, 0xffff);
    urx->_write_oe(which, (POWER_UP|RX2_RX1N|ENABLE), 0xffff);
    urx->write_io(which, (power_on|RX2_RX1N|ENABLE), (POWER_UP|RX2_RX1N|ENABLE));

    set_auto_tr(false);

    float gmax, gmin, gstep;
    get_gain_range(gmin, gmax, gstep);
    set_gain((gmin + gmax) / 2.0);

} 

void db_flex1800_rx::get_gain_range(float &gmin, float &gmax, float &gstep)
{
    get_pga_gain_range(gmin, gmax, gstep);
    gstep = 0.05;
    gmax += 70;
}

db_flex1800_tx::db_flex1800_tx(usrp_standard_tx *_u, unsigned int _w) : db_flex(_u, _w, &vco1800)
{
    power_on = ~POWER_UP;
    power_off = ~POWER_UP;  // Powering it off kills the serial bus

    std::cout << "In db_flex TX constructor which = " << which << " power on = " << power_on << std::endl;
    utx->_write_oe(which, 0, 0xffff);
    utx->_write_oe(which, (POWER_UP|RX_TXN|ENABLE), 0xffff);
    utx->write_io(which, (power_on|RX_TXN), (POWER_UP|RX_TXN|ENABLE));

    set_auto_tr(false);
    float gmax, gmin, gstep;
    get_gain_range(gmin, gmax, gstep);
    set_gain(gmax);

} 

void db_flex1800_tx::get_gain_range(float &gmin, float &gmax, float &gstep)
{
   get_pga_gain_range(gmin, gmax, gstep);
   gmin = gmax;
   gstep = 1.0;
}

db_flex1800_rx_mimo_a::db_flex1800_rx_mimo_a(usrp_standard_rx *_u, unsigned int _w) : db_flex1800_rx(_u, _w)
{
    vco1800.setR_DIV(1);
    vco1800.setREFCLK_DIVISOR(16);         
} 

db_flex1800_tx_mimo_a::db_flex1800_tx_mimo_a(usrp_standard_tx *_u, unsigned int _w) : db_flex1800_tx(_u, _w)
{
    vco1800.setR_DIV(1);
    vco1800.setREFCLK_DIVISOR(16);
}

db_flex1800_rx_mimo_b::db_flex1800_rx_mimo_b(usrp_standard_rx *_u, unsigned int _w) : db_flex1800_rx(_u, _w)
{
         
} 

db_flex1800_tx_mimo_b::db_flex1800_tx_mimo_b(usrp_standard_tx *_u, unsigned int _w) : db_flex1800_tx(_u, _w)
{

}
*/

db_flex2400_rx::db_flex2400_rx(usrp_standard_rx *_u, unsigned int _w) : db_flex(_u, _w, &vco2400)
{
    power_on = ~POWER_UP;
    power_off = ~POWER_UP;  // Powering it off kills the serial bus

    urx->_write_oe(which, 0, 0xffff);
    urx->_write_oe(which, (POWER_UP|RX2_RX1N|ENABLE), 0xffff);
    urx->write_io(which, (power_on|RX2_RX1N|ENABLE), (POWER_UP|RX2_RX1N|ENABLE));

    set_auto_tr(false);

    float gmax, gmin, gstep;
    get_gain_range(gmin, gmax, gstep);
    set_gain((gmin + gmax) / 2.0);

} 

void db_flex2400_rx::get_gain_range(float &gmin, float &gmax, float &gstep)
{
    get_pga_gain_range(gmin, gmax, gstep);
    gstep = 0.05;
    gmax += 70;
}

db_flex2400_tx::db_flex2400_tx(usrp_standard_tx *_u, unsigned int _w) : db_flex(_u, _w, &vco2400)
{
    power_on = ~POWER_UP;
    power_off = ~POWER_UP;  // Powering it off kills the serial bus

    DEBUG(3, USRP, "In db_flex TX constructor which = " << which << " power on = " << power_on)
    utx->_write_oe(which, 0, 0xffff);
    utx->_write_oe(which, (POWER_UP|RX_TXN|ENABLE), 0xffff);
    utx->write_io(which, (power_on|RX_TXN), (POWER_UP|RX_TXN|ENABLE));

    set_auto_tr(false);
    float gmax, gmin, gstep;
    get_gain_range(gmin, gmax, gstep);
    set_gain(gmax);

} 

void db_flex2400_tx::get_gain_range(float &gmin, float &gmax, float &gstep)
{
   get_pga_gain_range(gmin, gmax, gstep);
   gmin = gmax;
   gstep = 1.0;
}

db_flex2400_rx_mimo_a::db_flex2400_rx_mimo_a(usrp_standard_rx *_u, unsigned int _w) : db_flex2400_rx(_u, _w)
{
    vco2400.setR_DIV(1);
    vco2400.setREFCLK_DIVISOR(16);         
} 

db_flex2400_tx_mimo_a::db_flex2400_tx_mimo_a(usrp_standard_tx *_u, unsigned int _w) : db_flex2400_tx(_u, _w)
{
    vco2400.setR_DIV(1);
    vco2400.setREFCLK_DIVISOR(16);
}

db_flex2400_rx_mimo_b::db_flex2400_rx_mimo_b(usrp_standard_rx *_u, unsigned int _w) : db_flex2400_rx(_u, _w)
{
         
} 

db_flex2400_tx_mimo_b::db_flex2400_tx_mimo_b(usrp_standard_tx *_u, unsigned int _w) : db_flex2400_tx(_u, _w)
{

}

