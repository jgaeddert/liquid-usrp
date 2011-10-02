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

1.  liquid-dsp  :   liquid digital signal processing library, available
                    from github: http://github.com/jgaeddert/liquid-dsp
    
    $ git clone git://github.com/jgaeddert/liquid-dsp.git
    $ cd liquid-dsp
    $ ./reconf
    $ ./configure
    $ make
    $ sudo make install

2.  uhd         :   universal hardware driver from Ettus research,
                    available from http://www.ettus.com
    
    install dependencies using apt-get: cmake python-cheetah libusb-1.0-0-dev
                                        libboost-1.37-dev
                                        libboost-math1.37-dev

    install dependencies using port: cmake py-cheetah (py26-cheetah)

    $ git clone git://code.ettus.com/ettus/uhd.git
    $ cd uhd
    $ git checkout release_003_000_001
    $ cd host
    $ mkdir build
    $ cd build
    $ cmake ../
    $ make
    $ sudo make install


Build all the liquid-usrp example programs
    $ ./reconf
    $ ./configure
    $ make


