
liquid-usrp : Software-Defined Radio, USRP applications
=======================================================

liquid-dsp is a free and open-source digital signal processing (DSP) 
library designed specifically for software-defined radios on embedded 
platforms. The aim is to provide a lightweight DSP library that does not 
rely on a myriad of external dependencies or proprietary and otherwise 
cumbersome frameworks.

liquid-usrp contains simple applications for communicating over the air
using the universal software radio peripheral (USRP). Furthermore, a 
simple packet radio interface is presented that allows the user to 
transmit data packets of any length, modulation scheme, and 
forward error-correction scheme desired.

For more information on the USRP itself, refer to http://www.ettus.com/.

## Build Guide ##

  1. Install liquid-dsp (HEAD revision) digital signal processing
     library, available from github at http://github.com/jgaeddert/liquid-dsp
        
        $ git clone git://github.com/jgaeddert/liquid-dsp.git
        $ cd liquid-dsp
        $ ./bootstrap.sh
        $ ./configure
        $ make
        $ make check                    # run optional tests
        $ sudo make install

  2. Build the universal hardware driver (UHD) from Ettus research,
     available from http://www.ettus.com. Follow the official install
     guide: http://code.ettus.com/redmine/ettus/projects/uhd/wiki

  3. Build all the liquid-usrp example programs

        $ ./bootstrap.sh
        $ ./configure
        $ make

