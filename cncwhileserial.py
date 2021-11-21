#__author:StevenChen
#__Date:2021/11/20
# -*- coding: utf-8 -*-
import serial
import re
import time
import sys
import argparse
import threading
serialPort="com4"
baudRate = 9600
ser = serial.Serial(serialPort,baudRate,timeout=0.5)    ###serial = serial.Serial(serialPort,baudRate,timeout=0.5)


#ser.write(b'\r\n\r\n')
time.sleep(2)
while True:
    a = input('shuru')
    ser.write((str(a) + '\r\n').encode())
    if ser.in_waiting:
           line = ser.readlines()
           print(line)

           #serial.write("x_center".encode())#ser.write在于向串口中写入数据
           #line = ser.readlines()
           #print(line)
           # sensor = float(sensor)
           # output = float(output)


           #ser.write(b'$')
           #ser.write(b'$$')
           #print(ser.in_waiting)
           #line = ser.readall()
           #print(line)

           #ser.write(b'G01X10')
           #line = ser.readlines()
           #print(line)

ser.close()

