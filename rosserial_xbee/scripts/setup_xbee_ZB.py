#!/usr/bin/env python2

from xbee import ZigBee

import serial
import yaml
import sys
import time 
import argparse
import struct

baud_rates = [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]

def rxCallback(msg):
    print msg
    return

def open_serial_port(port, baud_rate,timeout):
    # Open serial port
    ser = serial.Serial(port, baud_rate, timeout=timeout)
    ser.flush()
    ser.flushInput()
    ser.flushOutput()
    time.sleep(0.1)
    return ser



parser = argparse.ArgumentParser("Setup the Zigbee radio for rosserial")
parser.add_argument('port', metavar='/dev/XXXX', type=str, help="Serial port to which the ZigBee radio is connected")
parser.add_argument('baud_rate', metavar='XXXX', type=str, help="Operating baud_rate of the radio")
parser.add_argument('--ni', metavar='NAME', type=str, help='Name of the zigbee radio in this network')
parser.add_argument('--id', metavar='\\xXX\\xXX', type=str, help='A 2-byte string representing the network ID of the ZigBee network')
parser.add_argument('--bd', metavar='\\xXX\\xXX', type=int, help='New baud rate desired for the radio. Numbers 0 to 7 correspond to 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200')

args = parser.parse_args()

# Open serial port
ser = open_serial_port(args.port, args.baud_rate, timeout= 0.1)
# Create API object
xbee = ZigBee(ser, callback= rxCallback,  escaped= True)

print "Started Xbee"
if args.id is not None:
    print 'Setting ID to %s'%(args.id)
    xbee.send('at', command='ID', parameter=args.id)
    time.sleep(0.5)

if args.ni is not None:
    print 'Setting NI to %s'%(args.ni)
    xbee.send('at', command='NI', parameter=args.ni)
    time.sleep(0.5)

if args.bd is not None:
    bd = args.bd
    if args.bd>7:
        pass
    elif args.bd <= 7:
        bd = baud_rates[args.bd]
        print 'Setting BD to %s'%(bd)
        xbee.send('at', command='BD', parameter=struct.pack(">H",args.bd))
        time.sleep(0.5)
        # Open serial port
        ser = open_serial_port(args.port, '%d'%(bd), timeout=0.1)
        # Create API object
        xbee = ZigBee(ser, callback= rxCallback,  escaped= True)

xbee.send('at', command='AC')
time.sleep(0.5)

xbee.send('at', command='WR')
time.sleep(0.5)
time.sleep(0.5)

print "Wrote changes to radio"
xbee.halt()
ser.close()
