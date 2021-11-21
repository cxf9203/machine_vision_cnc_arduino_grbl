#__author:StevenChen
#__Date:2021/11/21
#这脚本通过加载G代码文件向grbl控制器发送G代码指令
#注意事项：发送时信息与反馈信息打印不一致，但是信息是已经发送过去了。
import serial
import time

f=open('gcode.txt','r')
serialPort="com4"
baudRate = 9600
ser = serial.Serial(serialPort,baudRate,timeout=0.5)    ###serial = serial.Serial(serialPort,baudRate,timeout=0.5)


#ser.write(b'\r\n\r\n')
time.sleep(2)
ser.flushInput()
#TODO:
#MODIFIY AND IMPROVE SERIAL COMMUNICATE
for line in f:
    l=line.strip()
    print("sending "+l)
    ser.write((l+'\r\n').encode())
    grbl_res = ser.readlines()
    print(grbl_res)
while True:
    a = input('shuru')
    ser.write((str(a) + '\r\n').encode())
    if ser.in_waiting:
           line = ser.readlines()
           print(line)
f.close()
ser.close()