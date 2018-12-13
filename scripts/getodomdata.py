#!/usr/bin/env python
# -*- coding: utf-8 -*-


import time
import serial
import threading
import struct
import binascii

rcmd = [0x01, 0x40, 0x63, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc]
lcmd = [0x02, 0x40, 0x63, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb]
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)


def serialthread():
    ldata = struct.pack("%dB" % (len(lcmd)), *lcmd)
    rdata = struct.pack("%dB" % (len(rcmd)), *rcmd)
    ser.write(ldata)
    ser.write(rdata)
    rxData = ''
    while True:
        if ser.inWaiting() >= 20:
            rxData = ser.read(20)
            break
    return rxData


def odomdataprocess():
    odomdata = serialthread()
    lodom = odomdata[5:9]
    rodom = odomdata[15:19]
    # lodom = binascii.unhexlify(lodom)
    # rodom = binascii.unhexlify(rodom)
    lodom = struct.unpack('i', lodom)[0]
    rodom = struct.unpack('i', rodom)[0] * (-1)
    # print lodom
    # print rodom
    # rodom = '\xb4\xa2\xfe\xff'
    return lodom, rodom


if __name__ == "__main__":
    aa, bb = odomdataprocess()
    print aa, bb
