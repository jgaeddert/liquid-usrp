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

#include "ossie/debug.h"

#include "db_base.h"
#include "dbsrx.h"

//NEED TO FIGURE THIS ONE OUT####################################################
dbsrx::dbsrx(usrp_standard_rx *_urx, unsigned int _w):db_base(_urx, _w)
{
    u = _urx;
    lo_offset = -8E6;
    urx->_write_oe(which,0x0001,0x0001); //urx->_write_oe(which, CLOCK_OUT, CLOCK_OUT);

    if (which == 0){
        i2c_addr = 0x67;
    }
    else if (which == 1){
        i2c_addr = 0x65;
    }
    //int status_info;
    //int status_filter_DAC;
    n = 950;
    div2 = 0;
    osc = 5;
    cp = 3;
    r = 4;
    r_int = 1;
    fdac = 127;
    m = 2;
    dl = 0;
    ade = 0;
    adl = 0;
    gc2 = 31;
    diag = 0;
    /* 
    switch (slot) {
        case 0:
        ref_clkreg = 40;
        break;
        case 1:
        ref_clkreg = 41;
        break;
        case 2:
        ref_clkreg = 42;
        break;
        case 3:
        ref_clkreg = 43;
        break;
        default:
        std::cout << "DBSRX Error, Can not assign slot for DBSRX db" << std::endl;
        return;
    }
    */
    //std::cout << "slot is " << slot << std::endl;
    ref_clkreg = slot + 40;    
    bypass_adc_buffers(true);
    DEBUG(3, USRP, "bypassed adc buffers")
    
    float bw_min, bw_max;
    std::cout<< "getting bw range" <<std::endl;
    bw_range(bw_min, bw_max);
    std::cout<< "got bw range" <<std::endl;
    float init_bw;
    init_bw = bw_max;//(bw_max + bw_min)/2;
    std::cout <<"setting bw"<<std::endl;
    set_bw(init_bw);
    std::cout <<"set bw" <<std::endl;
    
    //enable_refclk(true); //not implemented in db_base class
    //u->_write_fpga_reg(40 + slot, 1); //Enable refclk?
    urx->_write_fpga_reg(ref_clkreg, (refclk_divisor() & REFCLK_DIVISOR_MASK) | REFCLK_ENABLE);//0x90
    
    
}

//NEED TO FIGURE THIS ONE OUT####################################################
dbsrx::~dbsrx()
{
    //enable_refclk(false); //not implemented in db_base class    
    //u->_write_fpga_reg(40 + slot, 0); //Disable refclk?
    urx->_write_fpga_reg(ref_clkreg, 0);
}

bool dbsrx::set_db_freq(float lo_freq, float &actual_freq)
{
    float rmin = std::max((float)2, (float) (refclk_freq()/2E6));
    float rmax = std::min((float)128, (float) (refclk_freq()/150E3));
    r = 2;
    n = 0;
    double delta;
    float best_r = 2;
    float best_n = 0;
    float best_delta = 10E6;
    int status_info;
    int status_filter_DAC;
    int vco;
    int adc_val = 0;
    float vco_freq;
    lo_freq = lo_freq - lo_offset;
    if(lo_freq < 500E6){
        actual_freq = 0;
        std::cerr << "DBSRX freq setting failed" << std::endl;
        std::cerr << "Requested lo_freq was " << lo_freq << std::endl;
        return false;
    }
    else if(lo_freq > 2.6E9){
        actual_freq = 0;
        std::cerr << "DBSRX freq setting failed" << std::endl;
        std::cerr << "Requested lo_freq was " << lo_freq << std::endl;
        return false;
    }
    else{
    }
    DEBUG(3, USRP, "lo_freq is " << lo_freq)
    if(lo_freq < 1150E6){
        set_div2(0);
        vco_freq = 4 * lo_freq;
    }
    else{
        set_div2(1);
        vco_freq = 2 * lo_freq;
    }
    while(r <= rmax){
        n = (int) round(lo_freq/(refclk_freq()/r));
        //std::cout << "n is " << n <<std::endl;
        if (r < rmin || n < 256){
            r = 2*r;
            continue;
        }
        delta = ((n * (refclk_freq()/r)) - lo_freq);
        //std::cout << "delta is " << delta << std::endl;
        if (delta < 0){
            delta = -1*delta;//Had to do this to get around a compiler complaint about using the abs() function
        }
        //std::cout << "delta is " << delta << std::endl;
        if (delta < 75E3){
            best_r = r;
            best_n = n;
            break;
        }
        //std::cout << "best_delta*0.9 is " << best_delta*0.9 << std::endl;
        if (delta < best_delta*0.9){
            best_r = r;
            best_n = n;
            best_delta = delta;
        }
        //std::cout << "best_r is " << best_r << std::endl;
        r = 2*r;
    }
    DEBUG(5, USRP, "(int) best_r is " << (int) best_r)
    set_r((int) best_r);
    //std::cout << "r is " << r << std::endl;
    //std::cout << "r_int is " << r_int << std::endl;
    //std::cout << "best_n is " << best_n <<std::endl;
    DEBUG(3, USRP, "(int) round(best_n) is " << (int) round(best_n))
    set_n((int) round(best_n+1));// +1 added to deal with ddc setting
    if(vco_freq < 2433E6){
        vco = 0;
    }
    else if(vco_freq < 2711E6){
        vco = 1;
    }
    else if(vco_freq < 3025E6){
        vco = 2;
    }
    else if(vco_freq < 3341E6){
        vco = 3;
    }
    else if(vco_freq < 3727E6){
        vco = 4;
    }
    else if(vco_freq < 4143E6){
        vco = 5;
    }
    else if(vco_freq < 4493E6){
        vco = 6;
    }
    else {
        vco = 7;
    }

    DEBUG(5, USRP, "vco is " << vco)
    set_osc(vco);
    //set_ade(1);
    adc_val = 0;
    while (adc_val==0||adc_val==7){
            //sleep(0.1);
            //adl = 0;
            set_ade(1);
            //adl = 0;
            //set_ade(1);            
            read_status(status_info, status_filter_DAC); //THIS MIGHT NEED TO CHANGE
            //adl = 0;
            //set_ade(1);
            adc_val = (status_info>>2);
            DEBUG(3, USRP, "adc_val is " << adc_val)
            if (adc_val==0){
                if (vco <=0){
                    actual_freq = 0;
                    std::cerr << "DBSRX freq setting failed:VCO under range" << std::endl;
                    std::cerr << "adc_val = " << adc_val << " and vco = " << vco << std::endl;
                    return false;
                }
                else {
                    vco = vco - 1;
                }
            }
            else if (adc_val==7){
                if (vco >= 7){
                    actual_freq = 0;
                    std::cerr << "DBSRX freq setting failed:VCO over range" << std::endl;
                    std::cerr << "adc_val = " << adc_val << " and vco = " << vco << std::endl;
                    return false;
                }
                else {
                    vco = vco + 1;
                }
            }
            set_osc(vco);
        }        
        if (adc_val==1||adc_val==2){
            set_cp(1);
        }
        else if (adc_val==3||adc_val==4){
            set_cp(2);
        }
        else {
            set_cp(3);
        }
        
    actual_freq = n*(refclk_freq()/r);
    std::cout << "Actual Baseband Frequency is " << (long) actual_freq << std::endl;
    DEBUG(3, USRP, "DBSRX Frequency Setting Successful")
    DEBUG(3, USRP, "Passband Frequency is " << (long) lo_freq)
    actual_freq = lo_freq - (actual_freq-lo_freq);// added to deal with ddc setting
        DEBUG(3, USRP, "Actual Baseband Frequency is " << (long) actual_freq)
    return true;
}


void dbsrx::set_gain(float gain)
{    
    int gc1 = 0;
    int gc2 = 0;
    int dl = 0;
    int pga = 0;
    //should do an assert here to be consistent
    if (gain<0||gain>105){
        std::cerr << "DBSRX Gain Value Out of Range, gain setting failed" <<std::endl;
        std::cerr << "Requested gain was " << gain << std::endl;
        return;
    }
    if (gain < 56){
        gc1 = (int) round(((-1*gain*1.85/56.0)+2.6)*(4096.0/3.3));
        gain = 0;
    }
    else {
        gc1 = 0;
        gain = gain - 56;
    }
    if (gain < 24){
        gc2 = (int) round(31.0 * (1-(gain/24)));
        gain = 0;
    }
    else {
        gc2 = 0;
        gain = gain -24;
    }
    if (gain >=4.58){
        dl = 1;
        gain = gain -4.58;        
    }
    pga = (int) round(gain);
    set_gc1(gc1);
    set_gc2(gc2);
    set_dl(dl);
    set_pga(pga);
    DEBUG(3, USRP, "DBSRX Gain Setting Successful")
    DEBUG(3, USRP, "pga gain set to " << pga)
    DEBUG(3, USRP, "gc1 is set to " << gc1)
    DEBUG(3, USRP, "gc2 is set to " << gc2)
}

void dbsrx::set_bw(float bw)
{
    int m_max;
    int m_min;
    int m_test;
    int fdac_test(0);
    float bw_min, bw_max;
    bw_range(bw_min, bw_max);
    //should probably use an assert here to be consistent
    if (bw<bw_min||bw>bw_max){
        std::cerr << "DBSRX BW setting out of range: Failed to Set BW" << std::endl;
        std::cerr << "Requested BW was " << bw << std::endl;
        return;
    }
    if (bw>=bw_min){
        m_max = (int) (std::min((double)31, floor(refclk_freq()/1E6)));
    }
    else if (bw>=bw_min/2){
        m_max = (int) (std::min((double)31, floor(refclk_freq()/5E5)));
    }
    else {
        m_max = (int) (std::min((double)31, floor(refclk_freq()/25E4)));
    }
    m_min = (int) (ceil(refclk_freq()/2.5E6));
    m_test = m_max;
    while (m_test>=m_min){
        fdac_test = (int) (round((((bw*m_test)/refclk_freq())-4)/.145));
        if (fdac_test>127){
            m_test = m_test - 1;
        }
        else {
            break;
        }
    }
        if (m_test>=m_min&&fdac_test>=0){
            set_m(m_test);
            set_fdac(fdac_test);
        }
        else {
            std::cerr << "DBSRX failed to set BW" << std::endl;
            std::cerr << "fdac_test was " << fdac_test << std::endl;
            std::cerr << "m_test was " << m_test << std::endl;
            return;
        }            
        DEBUG(3, USRP, "DBSRX LPF setting successful")
        DEBUG(3, USRP, "LPF BW set to " << bw)
}
void dbsrx::bw_range(float &min, float &max)
{
    min = 4E6;
    max = 33E6;
}    

void dbsrx::get_freq_range(float &min, float &max, float &step)
{
    min = 500000000.0;
    max = 2600000000.0;
    step = 1000000.0;
   
}

void dbsrx::get_gain_range(float &min, float &max, float &step)
{
    min = 0;
    max = 104;
    step = 1;
}

void dbsrx::write_reg(int regno, int value)
{
    DEBUG(5, USRP, "In dbsrx write_req with i2c address: " << i2c_addr << ", regno: " << regno << ", value: " << value);
    bool success;
    assert(regno>=0 &&regno<=5);
    std::string str(2,0);
    str[0] = (unsigned char) regno;
    str[1] = (unsigned char) value;
    success = urx->write_i2c(i2c_addr, str);
    if(false) {
            DEBUG(3, USRP, "str[0] is " << "####" << (int)((unsigned char*)str.data())[0] << "####")
            DEBUG(3, USRP, "str[1] is " << "####" << (int)((unsigned char*)str.data())[1] << "####")
            DEBUG(3, USRP, "size is " << str.length())
                DEBUG(3, USRP, "data is " << "####" << (unsigned char *)str.data() << "####")
        bool is_empty = str.empty();
        if(is_empty)
            std::cerr << "!!!!!!!!!!!!STRING IS EMPTY" << std::endl;
        if(success)
                        DEBUG(3, USRP, "Write to register " << regno << " with value " << value << " was successful.")
        else
            std::cerr << "Write to register " << regno << " with value " << value << " FAILED!!!!!!!!" << std::endl;
    }
        
}
//NEED TO FIGURE THIS ONE OUT####################################################
//void dbsrx::write_regs(int start_regno, int *values);
//{

//}
void dbsrx::read_status(int &status_byte1, int &status_byte2)
{
    std::string str;
    str = urx->read_i2c(i2c_addr, 2);
    if (str.length() != 2){
        status_byte1 = -1;
        status_byte2 = -1;
    }
    char buf[2];
    buf[0] = str[0];
    buf[1] = str[1];
    status_byte1 = (int) buf[0];
    status_byte2 = (int) buf[1];
    bool pwr_reset = (status_byte1>>5);
    int vco_voltage = ((status_byte1>>2)&0x07);
    int fdac_setting = status_byte2;
    if(pwr_reset)
        std::cerr << "Power was reset!" << std::endl;

    std::cout << "FDAC setting is " << fdac_setting << std::endl;
    DEBUG(3, USRP, "VCO ADC is " << vco_voltage)

    if(vco_voltage==0||vco_voltage==7)
        std::cerr << "VCO VOLTAGE IS OUT OF BOUNDS -> SOMEONE ISN'T PLAYING NICE -> REGISTER 0, THAT'S WHO!!!!" << std::endl;    
    else if(vco_voltage==1)
        DEBUG(3, USRP, "VCO voltage is approx 0.54 Volts")
    else if(vco_voltage==2)
        DEBUG(3, USRP, "VCO voltage is approx 0.785 Volts")
    else if(vco_voltage==3)
        DEBUG(3, USRP, "VCO voltage is approx 1.15 Volts")    
    else if(vco_voltage==4)
        DEBUG(3, USRP, "VCO voltage is approx 1.69 Volts")
    else if(vco_voltage==5)
        DEBUG(3, USRP, "VCO voltage is approx 2.475 Volts")    
    else if(vco_voltage==6)
        DEBUG(3, USRP, "VCO voltage is approx 3.72 Volts")
    else    
        std::cerr << "Something bad happened, I want my mommy!" << std::endl;
}

void dbsrx::send_reg(int regno)
{
    assert(regno>=0 &&regno<=5);
    if (regno==0) {
        write_reg(0,(((n>>8))|((div2<<7))));
        //std::cout << "write_reg(0) is " << (((n>>8))|((div2<<7))) << std::endl;
    }
    else if (regno==1) {
        write_reg(1,n&255);
        //std::cout << "write_reg(1) is " << (n&255) << std::endl;

    }
    else if (regno==2) {
        write_reg(2,osc|(cp<<3)|(r_int<<5));
        //std::cout << "write_reg(2) is " << (osc|(cp<<3)|(r_int<<5)) << std::endl;

    }
    else if (regno==3) {
        write_reg(3,fdac);
        //std::cout << "write_reg(3) is " << fdac << std::endl;

    }
    else if (regno==4) {
        write_reg(4,m|(dl<<5)|(ade<<6)|(adl<<7));
        //std::cout << "write_reg(4) is " << (m|(dl<<5)|(ade<<6)|(adl<<7)) << std::endl;

    }
    else if (regno==5) {
        write_reg(5,gc2|(diag<<5));
        //std::cout << "write_reg(5) is " << (gc2|(diag<<5)) << std::endl;

    }
}

//BW Setting Helper Function
void dbsrx::set_m(int m_in)
{
    assert(m_in>0&&m_in<32);    
    m = m_in;
    send_reg(4);
}
//BW etting Helper Function
void dbsrx::set_fdac(int fdac_in)
{
    assert(fdac_in>=0&&fdac_in<128);
    fdac = fdac_in;
    send_reg(3);
}

//Gain Setting Helper Function
void dbsrx::set_dl(int dl_in)
{
    assert(dl_in==0||dl_in==1);
    dl = dl_in;
    send_reg(4);
}

//Gain Setting Helper Function
void dbsrx::set_gc2(int gc2_in)
{
    assert(gc2_in<32&&gc2_in>=0);
    gc2 = gc2_in;
    send_reg(5);
}

//Gain Setting Helper Function
void dbsrx::set_gc1(int gc1_in)
{
    assert(gc1_in>=0&&gc1_in<4096);
    gc1 = gc1_in;
    bool write_aux_success;
    write_aux_success = urx->write_aux_dac(which,0,gc1);
}

//Gain Setting Helper Function
void dbsrx::set_pga(int pga_gain_in)
{
    assert(pga_gain_in>=0&&pga_gain_in<=20);
    if (which==0){
        set_pga_gain(0, pga_gain_in);
        set_pga_gain(1, pga_gain_in);
    }
    else {
        set_pga_gain(2, pga_gain_in);
        set_pga_gain(3, pga_gain_in);
    }    
}

//Frequency Setting Helper Funtion
void dbsrx::set_osc(int osc_in)
{
    assert(osc_in>=0&&osc_in<8);
    osc = osc_in;
    send_reg(2);    
}

//Frequency Setting Helper Funtion
void dbsrx::set_cp(int cp_in)
{
    assert(cp_in>=0&&cp_in<=3);
    cp = cp_in;
    send_reg(2);
}

//Frequency Setting Helper Funtion
void dbsrx::set_n(int n_in)
{
    assert(n_in>256&&n_in<32768);
    n = n_in;
    send_reg(0);
    send_reg(1);
}

//Frequency Setting Helper Funtion
void dbsrx::set_div2(int div2_in)
{
    assert(div2_in==0||div2_in==1);
    div2=div2_in;
}

//Frequency Setting Helper Funtion
void dbsrx::set_r(int r_in)
{
    assert(r_in>=0&&r_in<=127);
    r = r_in;
    r_int = (int) round(log10(r)/log10(2)-1);
    send_reg(2);
}

//Frequency Setting Helper Funtion
void dbsrx::set_ade(int ade_in)
{
    assert(ade_in==0||ade_in==1);
    ade=ade_in;
    send_reg(4);
}



