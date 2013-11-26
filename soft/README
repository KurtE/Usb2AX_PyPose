ParanoidStudio
==============

Firmware versions:
------------------
/lufa_usb2ax:         Default firmware, used to communicate with Dynamixels. TX and RX must be tied together (JP1 closed, which it is by default).
/lufa_usb2ax_serial:  USB to Serial firmware. TX and RX must be untied by cutting the trace between the pads of JP1.

How to install the firmware?
----------------------------
Upload the .hex file using Atmel FLIP.


How to build the firmware?
--------------------------
Inside LUFA::
 * Place the lufa_usb2ax/ directory in the Projects/ directory of LUFA
 * Execute `make all` in a terminal

Standalone::
 * Set LUFA_PATH to the location of LUFA on your computer
 * Execute `make all` in a terminal
 Example: `make LUFA_PATH=/opt/lufa all`


Required tools
--------------
 * avr-gcc
 * The LUFA AVR USB library

