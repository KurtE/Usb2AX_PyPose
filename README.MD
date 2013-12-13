Warning
#######

This is a Work In Progress!  There are no warrantees or Guarantees 
of any type that this code is useable for anything.  But I hope it is.

This is a hacked up version of the USB2AX project that I am trying to add  some
additional functionality.  Also I lot of the testing using a sparkfun atmega32u4
breakout board.  More on how to configure for this soon.

Note: I only have a subset of the files here from the original project.  I don't
have the schematics nor the test programs, nor the LUFA library.  You can grab
all of these files from the original project.

You can find out more information up at the website http://www.xevelabs.com/doku.php?id=product:usb2ax:usb2ax

The original git library is located at: https://paranoidstudio.assembla.com/code/paranoidstudio/git/nodes/master/usb2ax



ParanoidStudio
==============

USB2AX - Small USB to AX/MX-Series Dynamixel
-------------------------

Additional Functionality
========================

Fix Me!

How to update the USB2AX board
==============================

The update process relies on the Atmel Flip functionality.  You can download this from http://www.atmel.com/tools/FLIP.aspx
There is a command line program that is part of the flip proram that is used as one of the batch files for updating the firmware
on an USB2AX board. 

In order to reprogram the USB2AX board you need to get the board into the program mode. There is more information up at: 
http://www.xevelabs.com/doku.php?id=product:usb2ax:firmware_update

The main thing that is needed is to send a special packet to the USB2AX, that reboots the board and jumps to the bootloader. there is
a python script usb2ax_run_bootload.py in the soft\lufa_usb2ax folder that is setup to do this.  I have made some simple enhancements to
this script that allows me to pass in the comm port to this script.  So for example what I type on my machine to setup the USB2AXZ on my
machine to be in the bootloader is: python usb2ax_run_bootload.py com11

Once the script runs sucessfully, the USB2AX device will be rebooted and will shop up as a different type of device.  Once this is done you
can run the batch file prog.bat which should use the program that was installed as part of the flip program to install the hex file to the
USB2AX.  Once this completes the USB2AX will reboot and again show up as as comm port. 

How to use Sparkfun Breakout Board
==================================

As part of my testing, I have been using a couple of the Sparkfun Atmega32u4 break out boards.  The chips on these boards
are slightly different than the Atmega32u2, but have similar enough functionality and they have several IO pins which are
easy to use, which is useful when debugging. 

To make the board work the same, I replaced the boot loader that was shipped by Sparkfun, with the debug boot loader that
Atmel ships blank chips with.  You can download these boot loaders up at: http://www.atmel.com/devices/atmega32u4.aspx?tab=documents

I use the Tools/Device Program command in AtmelStudo (6.1).  I used an AVRJTAGICE MK2, but any support ISP should work.  

Once the bootloader is installed, I then install the firmware for the first time.  I use the python script to set the device into
program mode, typing a command like: python usb2ax_run_bootload.py com11
Note: I updated the python script to allow me to specify a com port on it, if one is not specified it will use the default on
in the script.

After this script runs, the device will reboot and it will show up with a different device type.  You can then program the 
device.  There are a couple of batch files in the software directory. prog.bat is the default script for the USB2AX and 
prog4 is for the atmega32u4 like the sparkfun board.

Once you have the program programed for the first time, it will not properly run this program, until you use the ISP again and
clear the bootrst fuse.

Note: when you switch between working with the USBTOAX device and one of the Atmega32u4 devices.  You need to change the device
type in the two makefiles in the soft directory and then rebuild the project. 

One of the nice things about having a JTAG device like the one I mentioned or an Atmel Dragon, is the ability to do source level
debugging on the device.  To do this you need to enable JTAG on the device, by setting the appropriate fuse. Then you need to
setup the correct wires.  For my JTAG device with it's squid wire harness (breaks out all 10 wires to their own jumpers), you can
setup the Sparkfun board something like:

Color   Signal  Pin
Black   TCK     PF4
White   GND     GND
Gray    TDO     PF6
Purple  VRef    VCC on ISP
Blue    TMS     PF5
Green   NSRST   Reset
Yellow  N/A
Orange  NTRST
Red     TDI     PF7
Brown   GND

Once you have this configured, you can hit F5 to start debugging, set breakpoints... Note: I believe that when I do this it 
completely erases the device.  So once I am completely done, I will probably need to reload the bootloader onto this device.

Probably still Fix Me!