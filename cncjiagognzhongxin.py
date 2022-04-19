import aliLink, mqttd
import paho.mqtt.client
import time, json, random
import hmac, hashlib
import wget
import os
import zipfile
import cv2 as cv
import numpy as np
import math
import threading
import serial
####################CNC GRBL######################################
##>>>>>>>>>>>>>>>>>>>> serial setting>>>>>>>>>>>>>>>>>>>>>>>>>>>>#
serialPort = "com3"
baudRate = 115200
ser = serial.Serial(serialPort, baudRate, timeout=0.5)
#########################com3 串口通信线程——from arduino##################
# create machine vision threading-1
class myThread(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
    def run(self):
        while 1:
            if ser.in_waiting:
                line = ser.readlines()
                out = line[1].decode('utf-8')
                print("get arduino info",out)

thread_vision = myThread(threadID=1, name="machinevision")
thread_vision.start()


######################### CNC G code programming cartissian space coordinate translation
def cncdrawcircle(x, y):#
    gcodecmd = "G01 " + "X" + str(x) + " Y" + str(y) + " F" + str(300) + '\r\n'
    gcodecmd += "G01 " + " Y" + str(100) + '\r\n'
    gcodecmd += "G02 " + "X" + str(100) + "R" + str(100) + '\r\n'
    gcodecmd += "G02 " + " Y" + str(-100) + "R" + str(100) + '\r\n'
    gcodecmd += "G02 " + " X" + str(-100) + "R" + str(100) + '\r\n'
    gcodecmd += "G02 " + " Y" + str(100) + "R" + str(100) + '\r\n'
    print("gcode cmd is :", gcodecmd)
    print("finish cnc drawing")
    # todo:
    # need a patten to milling
    ser.write((gcodecmd + '\r\n').encode())

############################IOT 远程回调信息#######################
sudu = 400
# 消息回调（云端下发消息的回调函数）
def on_message(client, userdata, msg):
    global sudu

    print(msg.payload)
    Msg = json.loads(msg.payload.decode('utf-8'))
    # callback Gcode
    if ('G_code_send' in Msg["params"].keys()):
        print("G_code_send received")
        Gcodetext = Msg['params']['G_code_send']
        print(Gcodetext)
        ser.write((Gcodetext + '\r\n').encode())
    else:
        print("G_code not received")

    # 上运动
    if ('shang' in Msg["params"].keys()):
        print("shang received")
        shang = Msg['params']['shang']

        if shang == 1:
            print("开始向上运动")
            gcodecmd = "G91 " + "G01 " + " Y" + str(10) + "F" + str(sudu)
        print("gcode cmd is :", gcodecmd)
        print("finish shang yundong")
        ser.write((gcodecmd + '\r\n').encode())
        #上报串口信息到云平台
        updateMsn = {
            # 'PowerLed':
            "textupload": "完成上运动"
        }
        JsonUpdataMsn = aliLink.Alink(updateMsn)
        print(JsonUpdataMsn)

        mqtt.push(POST, JsonUpdataMsn)  # 定时向阿里云IOT推送我们构建好的Alink协议数据
    else:
        pass
    # 下运动
    if ('xia' in Msg["params"].keys()):
        print("xia received")
        xia = Msg['params']['xia']

        if xia == 1:
            print("开始向下运动")
            gcodecmd = "G91 " + "G01 " + " Y" + str(-10) + "F" + str(sudu)
        print("gcode cmd is :", gcodecmd)
        print("finish cnc drawing")
        # todo:
        #   need a patten to milling
        ser.write((gcodecmd + '\r\n').encode())
        #上报串口信息到云平台
        updateMsn = {
            # 'PowerLed':
            "textupload": "完成下运动"
        }
        JsonUpdataMsn = aliLink.Alink(updateMsn)
        print(JsonUpdataMsn)

        mqtt.push(POST, JsonUpdataMsn)  # 定时向阿里云IOT推送我们构建好的Alink协议数据
    else:
       pass
    # 左运动
    if ('zuo' in Msg["params"].keys()):
        print("zuo received")
        zuo = Msg['params']['zuo']

        if zuo == 1:
            print("开始向左运动")
            gcodecmd = "G91 " + "G01 " + " X" + str(-10) + "F" + str(sudu)
        print("gcode cmd is :", gcodecmd)
        print("finish cnc drawing")
        # todo:
        #   need a patten to milling
        ser.write((gcodecmd + '\r\n').encode())
        #上报串口信息到云平台
        updateMsn = {
            # 'PowerLed':
            "textupload": "完成左运动"
        }
        JsonUpdataMsn = aliLink.Alink(updateMsn)
        print(JsonUpdataMsn)

        mqtt.push(POST, JsonUpdataMsn)  # 定时向阿里云IOT推送我们构建好的Alink协议数据
    else:
        pass
    # 右运动
    if ('you' in Msg["params"].keys()):
        print("you received")
        you = Msg['params']['you']

        if you == 1:
            print("开始向右运动")
            gcodecmd = "G91 " + "G01 " + " X" + str(10) + "F" + str(sudu)
        print("gcode cmd is :", gcodecmd)
        print("finish cnc drawing")
        # todo:
        #   need a patten to milling
        ser.write((gcodecmd + '\r\n').encode())
        #上报串口信息到云平台
        updateMsn = {
            # 'PowerLed':
            "textupload": "完成右运动"
        }
        JsonUpdataMsn = aliLink.Alink(updateMsn)
        print(JsonUpdataMsn)

        mqtt.push(POST, JsonUpdataMsn)  # 定时向阿里云IOT推送我们构建好的Alink协议数据
    else:
        pass

#左上运动
    if ('zuoshang' in Msg["params"].keys()):
        print("zuoshang received")
        zuoshang = Msg['params']['zuoshang']

        if zuoshang == 1:
            print("开始向左上运动")
            gcodecmd = "G91 " + "G01 " + " X" + str(-10)+" Y" + str(10)  + "F" + str(sudu)
        print("gcode cmd is :", gcodecmd)
        print("finish ZUOSHANG cnc drawing")
        # todo:
        #   need a patten to milling
        ser.write((gcodecmd + '\r\n').encode())
        #上报串口信息到云平台
        updateMsn = {
            # 'PowerLed':
            "textupload": "完成左上运动"
        }
        JsonUpdataMsn = aliLink.Alink(updateMsn)
        print(JsonUpdataMsn)

        mqtt.push(POST, JsonUpdataMsn)  # 定时向阿里云IOT推送我们构建好的Alink协议数据
    else:
        pass
# 右上运动
    if ('youshang' in Msg["params"].keys()):
        print("youshang received")
        youshang = Msg['params']['youshang']

        if youshang == 1:
            print("开始向右上运动")
            gcodecmd = "G91 " + "G01 " + " X" + str(10) + " Y" + str(10) + "F" + str(sudu)
        print("gcode cmd is :", gcodecmd)
        print("finish YOUSHANG cnc drawing")
        # todo:
        #   need a patten to milling
        ser.write((gcodecmd + '\r\n').encode())
        #上报串口信息到云平台
        updateMsn = {
            # 'PowerLed':
            "textupload": "完成右上运动"
        }
        JsonUpdataMsn = aliLink.Alink(updateMsn)
        print(JsonUpdataMsn)

        mqtt.push(POST, JsonUpdataMsn)  # 定时向阿里云IOT推送我们构建好的Alink协议数据
    else:
        pass
# 左下运动
    if ('zuoxia' in Msg["params"].keys()):
        print("zuoxia received")
        zuoxia = Msg['params']['zuoxia']

        if zuoxia == 1:
            print("开始向左下运动")
            gcodecmd = "G91 " + "G01 " + " X" + str(-10) + " Y" + str(-10) + "F" + str(sudu)
        print("gcode cmd is :", gcodecmd)
        print("finish zuoxia cnc drawing")
        # todo:
        #   need a patten to milling
        ser.write((gcodecmd + '\r\n').encode())
        #上报串口信息到云平台
        updateMsn = {
            # 'PowerLed':
            "textupload": "完成左下运动"
        }
        JsonUpdataMsn = aliLink.Alink(updateMsn)
        print(JsonUpdataMsn)

        mqtt.push(POST, JsonUpdataMsn)  # 定时向阿里云IOT推送我们构建好的Alink协议数据
    else:
        pass
 # 右下运动
    if ('youxia' in Msg["params"].keys()):
        print("youxia received")
        youxia = Msg['params']['youxia']

        if youxia == 1:
            print("开始向左下运动")
            gcodecmd = "G91 " + "G01 " + " X" + str(10) + " Y" + str(-10) + "F" + str(sudu)
        print("gcode cmd is :", gcodecmd)
        print("finish youxia cnc drawing")
        # todo:
        #   need a patten to milling
        ser.write((gcodecmd + '\r\n').encode())
        #上报串口信息到云平台
        updateMsn = {
            # 'PowerLed':
            "textupload": "完成右下运动"
        }
        JsonUpdataMsn = aliLink.Alink(updateMsn)
        print(JsonUpdataMsn)

        mqtt.push(POST, JsonUpdataMsn)  # 定时向阿里云IOT推送我们构建好的Alink协议数据
    else:
        pass
    # 速度调节
    if ('vel' in Msg["params"].keys()):
        print("velocity received")
        sudu = Msg['params']['vel']
        print("current velocity is :",sudu)
        #上报信息到云平台
        updateMsn = {
            # 'PowerLed':
            "textupload": "velocity set done "+str(sudu),
            "vel": sudu

        }
        JsonUpdataMsn = aliLink.Alink(updateMsn)
        print(JsonUpdataMsn)

        mqtt.push(POST, JsonUpdataMsn)
    else:
        pass

# 连接回调（与阿里云建立链接后的回调函数）
def on_connect(client, userdata, flags, rc):
    pass
#############################################
# 三元素（iot后台获取）
ProductKey = 'a1rmDbFqUEY'
DeviceName = 'machinevisionCNC'
DeviceSecret = "c2b9824ba2676f550cf1ddddb411d956"
# topic (iot后台获取)
POST = '/sys/a1rmDbFqUEY/machinevisionCNC/thing/event/property/post'  # 上报消息到云
POST_REPLY = '/sys/a1rmDbFqUEY/machinevisionCNC/thing/event/property/post_reply'
SET = '/sys/a1rmDbFqUEY/machinevisionCNC/thing/service/property/set'  # 订阅云端指令
####OTA TOPIC update####
#OTAVSEND = '/ota/device/inform/a1cQSE3paBi/PC-office' #topic IOT后台获取
#OTARECEIVE = '/ota/device/upgrade/a1cQSE3paBi/PC-office'
#OTAPROCESS = '/ota/device/progress/a1cQSE3paBi/PC-office'
# 链接信息
Server,ClientId,userNmae,Password = aliLink.linkiot(DeviceName,ProductKey,DeviceSecret)
# mqtt链接
mqtt = mqttd.MQTT(Server,ClientId,userNmae,Password)
mqtt.subscribe(SET) # 订阅服务器下发消息topic
mqtt.begin(on_message,on_connect)
#################################################
######################IOT END######################

# 上报初始信息到云平台
updateMsn = {
    # 'PowerLed':
    "vel": sudu

}
JsonUpdataMsn = aliLink.Alink(updateMsn)
print(JsonUpdataMsn)

mqtt.push(POST, JsonUpdataMsn)