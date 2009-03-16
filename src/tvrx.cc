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
#include <cassert>

#include <math.h>
#include <unistd.h>

#include "db_base.h"
#include "tvrx.h"

tvrx::tvrx(usrp_standard_rx *_urx, unsigned int _w):db_base(_urx, _w)
{
    if(!which)
        i2c_addr = 0x60;
    else
        i2c_addr = 0x61;
    reference_divisor = 640;
    fast_tuning = false;
    bypass_adc_buffers(false);
    //set defaults for rev 1
    set_inverted(true);
    set_first_IF_freq(43.75e6);
    set_second_IF_freq(5.75e6);
}

tvrx::~tvrx()
{

}

bool tvrx::set_db_freq(float lo_freq, float &actual_freq)
{
    float min, max, step;
    float target_lo_freq, actual_lo_freq, freq_ref;
    int divisor;
    bool write_i2c_success;
    std::string str(4,0);
    get_freq_range(min, max, step);
    //assert(min<=lo_freq && lo_freq<=max);
    if(lo_freq<min || lo_freq>max){
        actual_freq = 0;
        return false;    
    }
    
    target_lo_freq = lo_freq + first_if_freq();
    freq_ref = 4e6 / reference_divisor;
    
    divisor = (int) ((target_lo_freq + (4*freq_ref))/(8*freq_ref));
    actual_lo_freq = (8 * freq_ref * divisor);
    
    actual_freq = actual_lo_freq - first_if_freq();

    if((divisor&(~0x7fff))!=0){
        actual_freq = 0;
        return false;
    }
    
    str[0] = (unsigned char) ((divisor>>8)&0xff); 
    str[1] = (unsigned char) (divisor&0xff);
    str[2] = (unsigned char) (control_byte_1(fast_tuning, reference_divisor));
    str[3] = (unsigned char) (control_byte_2(actual_freq, false));

    write_i2c_success = urx->write_i2c(i2c_addr, str);

    actual_freq = actual_freq - second_if_freq();

    return write_i2c_success;
        

}

void tvrx::set_gain(float gain)
{
    float min, max, step;
    float rf_gain, if_gain, pga_gain;
    get_gain_range(min, max, step);
    //assert(min<=gain && gain<=max);
    if(gain<min){
        gain = min;
        std::cerr << "Gain below minimum, ...setting to minimum" << std::endl;
    }
    else if (gain>max){
        gain = max;
        std::cerr << "Gain above maximum, ...setting to maximum" << std::endl;
    }
    if(gain > 60){
        rf_gain = 60;
        gain = gain-60;
    }
    else{
        rf_gain = gain;
        gain = 0;
    }
    if(gain > 35){
        if_gain = gain;
        gain = gain - 35;
    }
    else{
        if_gain = gain;
        gain = 0;
    }
    pga_gain = gain;
    set_rfagc((int) rf_gain);
    set_ifagc((int) if_gain);
    set_pga((int) pga_gain);    
}

void tvrx::get_freq_range(float &min, float &max, float &step)
{
    min = 50e6;
    max = 860e6;
    step = 10e3;    
}

void tvrx::get_gain_range(float &min, float &max, float &step)
{
    min = 0;
    max = 115;
    step = 1;
}

void tvrx::set_inverted(bool inverted_in)
{
    INVERTED = inverted_in;
}

void tvrx::set_first_IF_freq(float first_IF_freq_in)
{
    FIRST_IF_FREQ = first_IF_freq_in;
}

void tvrx::set_second_IF_freq(float second_IF_freq_in)
{
    SECOND_IF_FREQ = second_IF_freq_in;
}

//Helper functions
char tvrx::control_byte_1(bool fast_tuning_present, unsigned int reference_divisor_in)
{
    unsigned char cb1 = 0x88;
    if(fast_tuning_present)
        cb1 = cb1|0x40;
    if(reference_divisor_in == 512)
        cb1 = (cb1|0x3) << 1;
    else if(reference_divisor_in == 640)
        cb1 = (cb1|0x0) << 1;
    else if(reference_divisor_in == 1024)
        cb1 = (cb1|0x1) <<1;
    else
        assert(0);
    return cb1;
}
char tvrx::control_byte_2(float target_freq, bool shutdown_tx_pga)
{
    unsigned char cb2;
    if(target_freq < 158e6)
        cb2 = 0xa0;
    else if(target_freq < 464e6)
        cb2 = 0x90;
    else
        cb2 = 0x30;
    if(shutdown_tx_pga)
        cb2 = cb2|0x80;
    return cb2;

}

//Gain helper functions
void tvrx::set_rfagc(unsigned int rf_gain_in)
{
    float voltage;
    int dacword;
    bool write_aux_dac_success;
    assert(0<=rf_gain_in && rf_gain_in <=60);
    if(rf_gain_in == 60) //might be able to get rid of this
        voltage = 4.0;
    else
        voltage = (rf_gain_in/60)*2.25 + 1.25;
    dacword = (int) (4096*((voltage/1.22)/3.3));
    assert(0<=dacword && dacword < 4096);
    write_aux_dac_success = urx->write_aux_dac(which, 1, dacword);        
}
void tvrx::set_ifagc(unsigned int if_gain_in)
{
    float voltage;
    int dacword;
    bool write_aux_dac_success;
    assert(0<=if_gain_in && if_gain_in <=35);
    voltage = 2.1*(if_gain_in/35.0)+1.4;
    dacword = (int) (4096*((voltage/1.22)/3.3));
    assert(0<=dacword && dacword < 4096);
    write_aux_dac_success = urx->write_aux_dac(which, 0, dacword);        
}
void tvrx::set_pga(unsigned int pga_in)
{
    assert(0<=pga_in && pga_in<=20);
    if(!which)
        urx->set_pga(0, pga_in);
    else
        urx->set_pga(2, pga_in);
}

tvrx_rev2::tvrx_rev2(usrp_standard_rx *_urx, unsigned int _w):tvrx(_urx, _w)
{
    set_inverted(false);
    set_first_IF_freq(44e6);
    set_second_IF_freq(20e6);

}

tvrx_rev3::tvrx_rev3(usrp_standard_rx *_urx, unsigned int _w):tvrx(_urx, _w)
{
    set_inverted(false);
    set_first_IF_freq(44e6);
    set_second_IF_freq(20e6);

}


