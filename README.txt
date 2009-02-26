Install basic USRP libraries:
    # apt-get install usrp usrp-firmware

Configure USB and change USRP permissions

To be able to run a waveform that uses the USRP you will either need to have root permissions or give access to your user to use the USRP. The process for each distribution is different but for Fedora 5 or 6: Define a group named usrp (as root):

    # /usr/sbin/groupadd usrp

Add a user to the group

    # /usr/sbin/usermod -a usrp [username]

or, if your copy of usermod does not support this syntax try

    # /usr/sbin/usermod -G usrp -a [username]

Create a file named /etc/udev/rules.d/10-usrp.rules with the following contents, and be sure to include the : in MODE:="0660" otherwise the default usb_device rule will override MODE to "0640". This also applies to the GROUP setting.

    # rule to grant read/write access on USRP to group named usrp.
    # to use, install this file in /etc/udev/rules.d as 10-usrp.rules
    ACTION=="add", BUS=="usb", SYSFS{idVendor}=="fffe", SYSFS{idProduct}=="0002",
    GROUP:="usrp", MODE:="0660"

Restart the udev daemon to enable he changes.

    # killall -HUP udevd

You can check if this is working by examining /dev/bus/usb after plugging in a USRP:

    $ ls -lR /dev/bus/usb

You should see a device file with group usrp and mode crw-rw----

Now restart your machine.

Troubleshooting

Here are a list of common problems users have encountered while compiling, or trying to run the USRP device in OSSIE waveforms.

Problem: When installing usrp-0.12 I get the following error when running ./configure

error: USRP requires libusb. usb.h not found, stop. See http://libusb.sf.net

Solution: You need the libusb development tools. They are easily installed via yum, viz.

    # yum install libusb-devel

Problem: When installing usrp-0.12 I get the following error when running ./configure

configure: error: USRP requires sdcc. sdcc not found, stop. See http://sdcc.sf.net

Solution: You need to install SDCC (see instructions above).

Problem: When running nodeBooter, I get the following error:

bin/USRP: error while loading shared libraries: libusrp.so.0: cannot open
shared object file: No such file or directory

Solution: As root, run

    # /sbin/ldconfig

Problem: When running nodeBooter:

usrp: failed to find usrp[0]

Solution:

Restart the machine

Problem: While running nodeBooter I get the following or similar error:

    usrp_open_interface:usb_claim_interface: failed interface 2
    could not claim interface 2: Device or resource busy
    usrp_basic_rx: can't open rx interface
    Failed to create usrp rx
    terminate called after throwing an instance of
    'CF::LifeCycle::InitializeError'
    Aborted

Solution: Disconnect USRP's USB cable, reconnect, and re-start nodeBooter.

