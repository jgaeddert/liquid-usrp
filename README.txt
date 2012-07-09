==============================================================================

    liquid-usrp : Software-Defined Radio, USRP applications

==============================================================================

liquid-dsp is a free and open-source digital signal processing (DSP) library
designed specifically for software-defined radios on embedded platforms. The
aim is to provide a lightweight DSP library that does not rely on a myriad of
external dependencies or proprietary and otherwise cumbersome frameworks.

liquid-usrp contains simple applications for communicating over the air
using the universal software radio peripheral (USRP).
Furthermore, a simple packet radio interface is presented that
allows the user to transmit data packets of any length, modulation scheme, and
forward error-correction scheme desired.

For more information on the USRP itself, refer to http://www.ettus.com/ and
http://gnuradio.org/.

Build dependencies

1.  liquid-dsp version 1.2.0
        liquid digital signal processing library, available from github at
        http://github.com/jgaeddert/liquid-dsp
    
    $ git clone git://github.com/jgaeddert/liquid-dsp.git
    $ cd liquid-dsp
    $ git checkout v1.2.0       # make sure to checkout the proper version
    $ ./reconf
    $ ./configure
    $ make
    $ sudo make install

     -- OR -- 
     
    download the 1.2.0 tarball...

    $ wget http://ganymede.ece.vt.edu/downloads/liquid-dsp-1.2.0.tar.gz
    $ tar -xvf liquid-dsp-1.2.0.tar.gz
    $ cd liquid-dsp-1.2.0
    $ ./reconf
    $ ./configure
    $ make
    $ sudo make install

2.  uhd
        universal hardware driver from Ettus research, available from
        http://www.ettus.com

    Install using guide:
        http://code.ettus.com/redmine/ettus/projects/uhd/wiki

Build all the liquid-usrp example programs
    $ ./reconf
    $ ./configure
    $ make


