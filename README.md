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

You can find out more information about the USB2AX product, up at the website 
http://www.xevelabs.com/doku.php?id=product:usb2ax:usb2ax

The official original git library is located at: https://paranoidstudio.assembla.com/code/paranoidstudio/git/nodes/master/usb2ax


ParanoidStudio
==============

USB2AX - Small USB to AX/MX-Series Dynamixel
-------------------------

The USB2AX device is a small board that you can plug into a USB port on your computer or other processor such as a Raspberry Pi,
that is used to interface to Robotis AX and MX level servos (ttl). The board sits on the AX buss with an address of 0xfd and reponds
to a subset and superset of the standard AX12 packets.  The board is built around an Atmel Atmega32u2 chip.

The AX12 packets are in the form: \<0xff\> \<0xff\> \<device id\> \<length\> \<instruction\> \<parameters...\> \<checksum\>

The Standard firmware supports some of the standard instructions: 
0x2 (Read Data) - Will return some information like model number and version
0x6 (Reset) - Resets the device

There were also some additional commands like:
0x8 (Bootload) - Used to set the device into a mode that you can update the firmware
0x84 (Sync read) - New command to read in a number of parameters from all of the servos


Additional Functionality added in this version
==============================================

This purpose of this project is to extend the functionality of the firmware on this board in order to try to offload more of the work
that the main processor has been doing in the past.  I will try to cover some of this in the following paragraphs.

The main functionality I wanted to achieve was to offload the pose interpolation code from the host to the USB2AX.  With most of our
robots, we did not trust using the AX-12 speed parameters to control how long it would take to move from one position to another position
over some specified time.  So the pose engine was written that would do this work.  It does this by calculating the difference from the
current position to the new desired goal position, it also had defined a logical frame time, so from the requested move time, it calculates
the number of frames.  From the number of frames and the delta move in goal positions, it calculates how much each servo should update per
frame.   So most of the current programs our written, such that the main code must call into the interpolation engine often enough such that
the next frame is output at the appropriate time.    So my goal was to be able to tell the USB2AX, about a new set of positions and a desired
time and then let it take care of the rest. 

Additional Parameters that you can Read and write (0x2 Read) (0x3 write)
------------------------------------------------------------------------
Many of the parameters that I added were to support the Interpolation code, but some others have been added for other reasons,
like the ability to set timeout values and also some support to monitor servo voltages.  Most of the new ones are RW,
although are some special functionality.

\#define USB2AX_REG_USB_SEND_TIMEOUT    24   // Timeout for USB sends in clock tics (2us)
\#define USB2AX_REG_USB_RECEIVE_TIMEOUT 25   // Timeout for USB Receives in clock tics
\#define USB2AX_REG_USART_RECEIVE_TIMEOUT 26 // How long to wait on USART (AX Buss) for servo to respond per character

\#define USB2AX_REG_VOLTAGE			   27	// Hack write-\> which servo 0xff all - read -> gets cached voltage level
\#define USB2AX_REG_VOLTAGE_FRAME_TIME 28    // How often to check for voltages in tics

\#define USB2AX_REG_POSE_FRAME_TIME	   29	// How long should each frame take in ms
\#define USB2AX_REG_POSE_INTERPOLATING  30   // Mostly Read only - how many servos are still moving
\#define USB2AX_REG_POSE_SIZE		   31   // How many servos should we support with the pose engine (0-31) default 31
\#define USB2AX_REG_POSE_ID_FIRST	   32   // ID of the first servo
... (32-62) are the ids of all of the servos.  We init to 1, 2, 3, 4, ...  

\#define USB2AX_REG_SLOT_CUR_POSE_FIRST  64  // Mostly Read-only - Can grab current position for each of the servos, not needed much.

New command: Read Pose(0x85)
-----------------------------

This command will have the AX-12 loop through each of the defined servos and read in their current position, which is mainly used
to initialize the pose engine.  No additional arguments.

New Command: Pose Command by IDs(0x86)
--------------------------------------

This command allows you to start up a pose for all or a subset of the servos that are defined as part of the pose. 
\<0xff\> \<0xff\> \<0xfd\> \<len\> \<0x86\> \<move time l\> \<move time h\> \<id1\> \<goal Low\> \<goal high\> \<id2\> \<goal 2 l\> \<goal 2 h\> ... \<chksum\>

The first two arguments that are passed to this command is the time the move should take in ms. 

This is followed by 3 bytes per servo that are associated with this move.  The first byte is the ID of the servo, 
the next two bytes are the new goal position \<goal low\> \<goal high\>

New Command: Pose Command by Servo Mask(0x87)
--------------------------------------

Another form of the Pose command is instead of each time specifying the servo ID each time, instead we use a bit mask 
to specify which of the logical pose slots we are specifying new positions for.
\<0xff\> \<0xff\> \<0xfd\> \<len\> \<0x86\> \<move time l\> \<move time h\> \<Mask byte 1 L\>\<Mask 2\>\<M3\>\<m4\>\<M5 H\> \<goal 1L\>\<goal 1H\>...\<chksum\>

Again 2 bytes for move time (low high)

Mask of servos.  Currently using 5 bytes as in AX buss can not have two bytes in a row with a value 0xff, so to avoid this only
using 7 bits of data per byte.  7 7 7 7 3

For each bit that is on in the mask, there needs to be a corresponding 2 bytes of data for the goal position for that slot.

The advantage of this format is for larger poses like a 3dof hexapod, where you are mostly outputting new positions for all
of the leg servos, the command is shorter than the IDs format(2+5+18*2 = 41) versus (2+3*18 = 56)

New Command: Abort Pose(0x88)
-----------------------------

Currently this command has no additional arguments and will stop the moving of all of the servos that are part of the pose engine.
Later may allow you to optionally send a set of servos to specifically stop.

New functionality: Voltage monitoring
-------------------------------------

As I am using a Lipo battery to run my servos, I do not want the battery to go below some certain voltage level.  Idealy it would be
great if we could optionally have a voltage divider on the device to directly monitor the voltage levels, but the Atmega32u2 does not
have an Analog to Digital converter.  So instead we ask one or more servos for their voltage level.  In the past I have had 
difficulties with asking some of the servos for the voltage as the code may ask, just as we are wanting to update the servo positions.

So I have added some functionality to have the AX12 ask a servo every so often for its current voltage, which I cache away.  
The code tries to make sure that the asking will be done such that it will not interfere with the start of the next servo
pose frame.

Currently this code is controlled by the register USB2AX_REG_VOLTAGE I mentioned above.  This code is sort-of a hack, 
but what you write to this register is the servo id you wish to monitor.  There is special code in place that if you set this to
0xff, it will cycle through each of the servos that is defined as part of the pose engine.

The read of this register, will return the current cached voltage in 10ths of a voltage. 

How to update the USB2AX board
==============================

The update process relies on the Atmel Flip functionality.  You can download this from http://www.atmel.com/tools/FLIP.aspx
There is a command line program that is part of the flip program that is used as one of the batch files for updating the firmware
on an USB2AX board. 

In order to reprogram the USB2AX board you need to get the board into the program mode. There is more information up at: 
http://www.xevelabs.com/doku.php?id=product:usb2ax:firmware_update

The main thing that is needed is to send a special packet to the USB2AX, that reboots the board and jumps to the bootloader. there is
a python script usb2ax_run_bootload.py in the soft\lufa_usb2ax folder that is setup to do this.  I have made some simple enhancements to
this script that allows me to pass in the comm port to this script.  So for example what I type on my machine to setup the USB2AXZ on my
machine to be in the bootloader is: python usb2ax_run_bootload.py com11

Once the script runs successfully, the USB2AX device will be rebooted and will shop up as a different type of device.  Once this is done you
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

Once the boot loader is installed, I then install the firmware for the first time.  I use the python script to set the device into
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
setup the correct wires.  For my JTAG device with its squid wire harness (breaks out all 10 wires to their own jumpers), you can
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
