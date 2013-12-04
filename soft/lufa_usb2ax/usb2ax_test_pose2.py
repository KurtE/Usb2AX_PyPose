#! /usr/bin/env python
import serial
import time
import thread
import threading

#This test application pings and moves servo #14. Use only on a servo with nothing mounted on it!
#Requires PySerial

com_port = "COM61"
servo = 21
servo2 = 22
baudrate = 1000000

AX_CMD_READ_DATA  = 0x02
AX_CMD_WRITE_DATA = 0x03
AX_CMD_POSE_IDS   = 0x86
AX_CMD_POSE_MASK  = 0x87

AX_MODEL_NUMBER_L =          0
AX_MODEL_NUMBER_H =          1
AX_VERSION        =          2

AX_TORQUE_ENABLE  =          24
AX_LED            =          25
AX_CW_COMPLIANCE_MARGIN  =   26
AX_CCW_COMPLIANCE_MARGIN =   27
AX_CW_COMPLIANCE_SLOPE   =   28
AX_CCW_COMPLIANCE_SLOPE  =   29
AX_GOAL_POSITION_L       =   30
AX_GOAL_POSITION_H       =   31

USBTOAX_ID      =        0xFD
AX_REG_POSE_INTERPOLATING = 30
AX_REG_POSE_SIZE		  = 31
AX_REG_POSE_ID_FIRST	  = 32  # Try with 32 of these to start off with.


ser = serial.Serial(com_port, baudrate)  # open serial port

stop_flag = threading.Event()
lock = threading.Lock()

def dumpStr(strToD):
    print "(", len(strToD), ")",
    for i in range (0, len(strToD)):
        print hex(ord(strToD[i])) ," ",
        if ((i % 10) == 9) :
            print "\n"
    print "\n"    

def read_and_print():
    global ser, stop_flag
    while not stop_flag.is_set():
        cbToRead = ser.inWaiting();
        if (cbToRead != 0) :
            lock.acquire()
            cbToRead = ser.inWaiting();
            strRead = ser.read(cbToRead)
            print ">",
            dumpStr(strRead)
            lock.release()
        time.sleep(0.01)
        
def writeReg(id, reg, val):
    checksum = ~((id + 4 + 3 + reg + val) % 256)
    strout = '\xFF\xFF'+chr(id)+'\x04\x03'+chr(reg)+chr(val)+chr(checksum & 0xff)
    ser.write(strout) #try to output all of string at once...
    lock.acquire()
    dumpStr(strout)
    lock.release();

def readReg(id, reg, cnt):
    checksum = ~((id + 4 + 2 + reg + cnt) % 256)
    strout = '\xFF\xFF'+chr(id)+'\x04\x02'+chr(reg)+chr(cnt)+chr(checksum & 0xff)
    ser.write(strout) #try to output all of string at once...
    lock.acquire()
    dumpStr(strout)
    lock.release();
            
#updated to do 2 servos
def doPoseMask(pos, pos2, time):
    pktlen = 2 + 2+5+4
    strout = '\xFF\xFF'+chr(USBTOAX_ID)+chr(pktlen)+chr(AX_CMD_POSE_MASK)+chr(time&0xff)+chr((time >> 8) & 0xff)
    strout += chr(3)+chr(0)+chr(0)+chr(0)+chr(0)
    strout += chr(pos & 0xff) + chr((pos >> 8) & 0xff)
    strout += chr(pos2 & 0xff) + chr((pos2 >> 8) & 0xff)
    checksum = 0
    for i in range (2,len(strout)) :
        checksum += ord(strout[i])
    strout += chr((~(checksum % 256)) & 0xff)
    ser.write(strout)
    lock.acquire()
    dumpStr(strout)
    lock.release();

def doPoseID(pos, time):
    pktlen = 2 + 2+1+2
    strout = '\xFF\xFF'+chr(USBTOAX_ID)+chr(pktlen)+chr(AX_CMD_POSE_IDS)+chr(time&0xff)+chr((time >> 8) & 0xff)
    strout += chr(servo)
    strout += chr(pos & 0xff) + chr((pos >> 8) & 0xff)
    checksum = 0
    for i in range (2,len(strout)) :
        checksum += ord(strout[i])
    strout += chr((~(checksum % 256)) & 0xff)
    ser.write(strout)
    lock.acquire()
    dumpStr(strout)
    lock.release();

        
              
plop = thread.start_new_thread(read_and_print, ())

# initialize pypose like data. Set for two servos
writeReg(USBTOAX_ID, AX_REG_POSE_SIZE, 2)

# set the 1st id to our servo
writeReg(USBTOAX_ID, AX_REG_POSE_ID_FIRST, servo)
writeReg(USBTOAX_ID, AX_REG_POSE_ID_FIRST+1, servo2)

ser.write('\xFF\xFF\x0E\x02\x01\xEE') #ping
time.sleep(0.05)

#ser.write('\xFF\xFF\x0E\x04\x03\x19\x01\xCF') #set torque
writeReg(servo, AX_TORQUE_ENABLE, 1)
writeReg(servo2, AX_TORQUE_ENABLE, 1)
time.sleep(0.1)


# now lets try to read a few registers.
readReg(USBTOAX_ID, AX_MODEL_NUMBER_L, 2)
readReg(USBTOAX_ID, AX_VERSION, 1)
readReg(USBTOAX_ID, AX_REG_POSE_SIZE, 1)
readReg(USBTOAX_ID, AX_REG_POSE_ID_FIRST, 1)
readReg(USBTOAX_ID, AX_REG_POSE_ID_FIRST+1, 1)

doPoseMask(512, 512, 500)
time.sleep(0.5)

try:
    doPoseMask(256, 768, 500)
    time.sleep(.5)
    while True:
        doPoseMask(768, 256, 500)
        time.sleep(.5)
        doPoseMask(256, 256, 500)
        time.sleep(.5)
        doPoseMask(256, 768, 500)
        time.sleep(.5)
except:
    stop_flag.set()
