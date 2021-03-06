#!/usr/bin/env python2.6

# This script send a command to the USB2AX to put it in bootloader mode.
# Upon running this script, the USB2AX LEDs should turn off and it should re-enumerate
# as ATmega32u2 DFU. It can then be re-programed using the GUI application Atmel FLIP
# or it underlying cli program batchisp.
#
# 
# Nicolas Saugnier - 2012

#************************************************************************************
# Please set COM_PORT to the name or path to the USB2AX you wish to put in bootloader mode
#Windows example
#COM_PORT = "COM7"
#Linux examples
#COM_PORT = "/dev/ttyACM0"
#COM_PORT = "/dev/serial/by-id/usb-Xevelabs_USB2AX_64033353031351600170-if00"

COM_PORT = "COM61"

#************************************************************************************
# TODO :
# - version that accepts cli args...
# - proper copyright notice
# - error handling (serial package, serial port...)

import serial, sys
if __name__ == "__main__":
    if len(sys.argv) > 1:
        COM_PORT = sys.argv[1]

print "Set Bootloader on: " + COM_PORT


bootload_cmd = ''.join(chr(x) for x in [0xff, 0xff, 0xfd, 0x02, 0x08, 0xf8])
ser = serial.Serial(COM_PORT)
ser.write(bootload_cmd)
ser.close()
