==============================================================================

    liquid-usrp : Software-Defined Radio, USRP applications library

==============================================================================

liquid-dsp is a free and open-source digital signal processing (DSP) library
designed specifically for software-defined radios on embedded platforms. The
aim is to provide a lightweight DSP library that does not rely on a myriad of
external dependencies or proprietary and otherwise cumbersome frameworks.

liquid-usrp is a simple interface between the universal software radio
peripheral (USRP) and your signal processing code.  It provides some
advantages over the basic usrp device interface by providing a stripped-down
API, gain normalization, and the ability to set any arbitrary sampling rate
you want.  Furthermore, a simple packet radio interface is presented that
allows the user to transmit data packets of any length, modulation scheme, and
forward error-correction scheme desired.

For more information on the USRP itself, refer to http://www.ettus.com/ and
http://gnuradio.org/.

Build dependencies

    liquid-dsp  :   liquid digital signal processing library, available
                    from github: http://github.com/jgaeddert/liquid-dsp

    usrp-0.12   :   currently liquid-usrp only supports the legacy usrp
                    code (grab from ftp.gnu.org/gnu/gnuradio/old/usrp-0.12)

============================================
  Installation : liquid-usrp
============================================

Build and install the library:
    $ ./reconf
    $ ./configure --enable-legacy
    $ make
    # make install

Build the example programs:
    $ make examples

Uninstall the library:
    # make uninstall

============================================
  Installation : usrp-0.12
============================================

Install basic USRP libraries from ftp.gnu.org/gnu/gnuradio/old/usrp-0.12.tar.gz

Configure USB and change USRP permissions

To be able to run a waveform that uses the USRP you will either need to
have root permissions or give access to your user to use the USRP. The
process for each distribution is different but for Fedora 5 or 6: Define
a group named usrp (as root):

    # /usr/sbin/groupadd usrp

Add a user to the group

    # /usr/sbin/usermod -a usrp [username]

or, if your copy of usermod does not support this syntax try

    # /usr/sbin/usermod -G usrp -a [username]

Create a file named /etc/udev/rules.d/10-usrp.rules with the following
contents, and be sure to include the : in MODE:="0660" otherwise the
default usb_device rule will override MODE to "0640". This also applies
to the GROUP setting.

    # rule to grant read/write access on USRP to group named usrp.
    # to use, install this file in /etc/udev/rules.d as 10-usrp.rules
    ACTION=="add", BUS=="usb", SYSFS{idVendor}=="fffe", SYSFS{idProduct}=="0002", GROUP:="usrp", MODE:="0660"

Restart the udev daemon to enable he changes.

    # killall -HUP udevd

You can check if this is working by examining /dev/bus/usb after
plugging in a USRP:

    $ ls -lR /dev/bus/usb

You should see a device file with group usrp and mode crw-rw----

Now restart your machine.

============================================
  Troubleshooting
============================================

Here are a list of common problems users have encountered while
compiling, or trying to run the USRP device in applications.

Problem: When installing usrp-0.12 I get the following error when
         running ./configure

    error: USRP requires libusb. usb.h not found, stop. See http://libusb.sf.net

Solution: You need the libusb development tools. They are easily
          installed via apt-get, viz.

    # apt-get install libusb-devel

Problem: When installing usrp-0.12 I get the following error when
         running ./configure

    configure: error: USRP requires sdcc. sdcc not found, stop. See http://sdcc.sf.net

Solution: You need to install SDCC (see instructions above).

Problem: When running an application, I get the following error:

    bin/USRP: error while loading shared libraries: libusrp.so.0: cannot open
    shared object file: No such file or directory

Solution: As root, run

    # /sbin/ldconfig

Problem: installing old(er) version of gnuradio gives error during configure:
    checking Python.h usability... no
    checking Python.h presence... no
    checking for Python.h... no
    configure: error: cannot find usable Python headers

Solution: for some reason configure fails if you don't have the fort77
          compiler.

    # apt-get install fort77

