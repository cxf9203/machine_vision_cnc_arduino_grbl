'''
最小外接圆方法，尝试使用最小矩形做切割
方法二：利用平移模板的方式，在找到圆时立即切除，并平移模板继续寻找圆，一旦找到，立即切除。
'''
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

# serial setting
serialPort = "com4"
baudRate = 115200
ser = serial.Serial(serialPort, baudRate, timeout=0.5)


# CNC G code programming
# cartissian space coordinate translation
def cncdrawcircle(x, y):
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
    # ser.write((gcodecmd + '\r\n').encode())


# initialization
###computer vision settings###
capture = cv.VideoCapture(0, cv.CAP_DSHOW)
cncstart = bool(0)
# leatherImage = "leather1.jpg"
CircleRadius = 100  # try other number: 50 200 etc..
forSteps = 20
areaTolerance = 5
radiusTolerance = 2
mat = np.float32([[1, 0, forSteps], [0, 1, forSteps]])  # 沿（x，y）方向移动，移动的距离是（forSteps，forSteps）的移动矩阵
font = cv.FONT_HERSHEY_SIMPLEX  # opencv字体格式
contouredge = 5  # segment circles
kernel = np.ones((3, 3), int)
ret, frame = capture.read()
time.sleep(2)
capture.release()
Ileather_gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
###create Odd Row Circles###
size = Ileather_gray.shape
print(size)
m = size[0]  # hight rows
n = size[1]  # width column
"""get template"""
#### Create Odd Row Circles###
CentreX = CircleRadius
CentreXArrayOdd = [CentreX]
for i in range(n):
    CentreX = CentreX + 2 * CircleRadius + contouredge  # plus 1 for contour
    if CentreX > (n - CircleRadius):
        break
    CentreXArrayOdd.append(CentreX)  # 修改数组并添加数
# print(CentreXArrayOdd)


###create even row Circles###
CentreX = 2 * CircleRadius
CentreXArrayEven = [CentreX]
for i in range(n):  ##column
    CentreX = CentreX + 2 * CircleRadius + contouredge  # plus 1 for contour
    if CentreX > (n - CircleRadius):
        break
    CentreXArrayEven.append(CentreX)  # 修改数组并添加数
# print(CentreXArrayEven)


###Create Odd Column Circles###
CentreY = CircleRadius
CentreYArrayOdd = [CentreY]

for i in range(m):  #####row
    CentreY = CentreY + math.ceil(
        contouredge + 2 * CircleRadius * 0.87 * 2)  # plus 1 for contour #math.sin(60) = 0.866!!!!!!60度
    if CentreY > m - CircleRadius:
        break;
    CentreYArrayOdd.append(CentreY)
# print(CentreYArrayOdd)

####Create Even Column Circles###
CentreY = CircleRadius + math.ceil(2 * CircleRadius * 0.87)  # plus 1 for contour #math.sin(60) = 0.866!!!!!! 60度
CentreYArrayEven = [CentreY]
for i in range(m):  ###row
    CentreY = CentreY + math.ceil(contouredge + 2 * CircleRadius * 0.87 * 2)  # plus 1 for contour
    if CentreY > m - CircleRadius:
        break
    CentreYArrayEven.append(CentreY)
# print(CentreYArrayEven)


###Create Circles on the Centres###
CentreOdd = []
for i in range(len(CentreYArrayOdd)):
    for j in range(len(CentreXArrayOdd)):
        CentreOdd.append((CentreXArrayOdd[j], CentreYArrayOdd[i]))
# print(CentreOdd)

CentreEven = []
for i in range(len(CentreYArrayEven)):
    for j in range(len(CentreXArrayEven)):
        CentreEven.append((CentreXArrayEven[j], CentreYArrayEven[i]))
# print(CentreEven)

###get Centre coordination in digital image###
Centre = CentreOdd + CentreEven  # get centre data
# Centre.append(CentreEven)  error
# print(Centre)
# print(len(Centre))
# get a black image
img_black = np.zeros((m, n, 3), np.uint8)

size_img_black = img_black.shape
# print(size_img_black)
# cv.line(img_black,(0,0),(n,m),(255,0,0),5)

for k in range(len(Centre)):
    cv.circle(img_black,
              Centre[k],
              CircleRadius,
              (0, 0, 255),
              -1)  ### -1>>the circle is filled with color
# cv.imshow("image_black",img_black)
###change to grayscale###
img_black_grayscale = cv.cvtColor(img_black, cv.COLOR_RGB2GRAY)

###change to black and white(binary)###
ret, img_black_binary = cv.threshold(img_black_grayscale, 10, 255, cv.THRESH_BINARY)
size_binary = img_black_binary.shape
# print(size_binary)
# cv.imshow("black_binary",img_black_binary)

sudu = 100


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

    if ('semiautomatic' in Msg["params"].keys()):
        print("semiautomatic received")
        global cncstart
        cncstart = Msg['params']['semiautomatic']
        print("current cncstart is ", cncstart)
        if cncstart == 1:
            print("semiautomatic laser cutting is begin")
            machinevisionprocess()
        else:
            print("cncstart is 0, the machine is stop(or other state)")
    else:
        print("semiautomatic not receive")

    # 上运动
    if ('shang' in Msg["params"].keys()):
        print("shang received")
        shang = Msg['params']['shang']

        if shang == 1:
            print("开始向上运动")
            gcodecmd = "G91 " + "G01 " + " Y" + str(10) + "F" + str(sudu)
        print("gcode cmd is :", gcodecmd)
        print("finish shang yundong")
        # todo:
        #   need a patten to milling
        ser.write((gcodecmd + '\r\n').encode())
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
    else:
        pass
    # 速度调节
    if ('sudu' in Msg["params"].keys()):
        print("G_code_send received")
        sudu = Msg['params']['sudu']
        print(sudu)
    else:
        pass




# 连接回调（与阿里云建立链接后的回调函数）
def on_connect(client, userdata, flags, rc):
    pass


def machinevisionprocess():
    global img_black_binary
    centre = []
    Ileather = thread_vision.get_result()
    Ileather_gray = cv.cvtColor(Ileather, cv.COLOR_RGB2GRAY)
    ret, Ileather_binary = cv.threshold(Ileather_gray, 127, 255, cv.THRESH_BINARY)
    for k in range(0, forSteps, 2 * CircleRadius):  # 嵌套循环去平移模板去遍历整张图象
        LeatherMultiplyCircles = cv.bitwise_and(img_black_binary, Ileather_binary, dst=None,
                                                mask=None)  # get multiply binary image
        LeatherMultiplyCircles = cv.erode(LeatherMultiplyCircles, kernel)  # 腐蚀进来缩小边界接触点

        ###Hough圆检测方法###
        contours, hierarchy = cv.findContours(LeatherMultiplyCircles, cv.RETR_EXTERNAL,
                                              cv.CHAIN_APPROX_NONE)  # 找到该对象外部轮廓
        minRect = [None] * len(contours)
        for i, c in enumerate(contours):
            minRect[i] = cv.minAreaRect(c)
        # print("minimum rectangle area",minRect)

        for i, c in enumerate(contours):
            box = cv.boxPoints(minRect[i])
            box = np.intp(box)
            box_w = minRect[i][1][0]  # get box2D structure,(x,y) and (w,h) from minRect
            box_h = minRect[i][1][1]
            if box_w * box_h > 30000:
                if box_w < box_h + 0.1 and box_w > box_h - 0.1:
                    cv.drawContours(Ileather_binary, contours, i, 0,
                                    -1)  # delete original binary then to get circle center
                    M = cv.moments(c)
                    # cv.imshow("LeatherMultiplyCirclesprocess", Ileather_binary)
                    centre_mid = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
                    centre.append(centre_mid)
            else:
                pass
        ##translate circle for next loop
        img_black_binary = cv.warpAffine(img_black_binary, mat,
                                         (img_black_binary.shape[1], img_black_binary.shape[0]))
    # print('circle centre x is :', centre_x, 'circle centre y is :', centre_y)
    print(centre)
    for i in range(len(centre)):
        centre_x = centre[i][0]
        centre_y = centre[i][1]
        cncdrawcircle(centre_x, centre_y)
        cv.circle(Ileather, (centre_x, centre_y), 7, (0, 255, 0), -1)  # 画出圆心
        cv.circle(Ileather, (centre_x, centre_y), CircleRadius, (0, 255, 0), 4)  # 画出半径轮廓
        cv.line(Ileather, (centre_x, centre_y), (centre_x + CircleRadius, centre_y), (0, 255, 0), 2)  # 画出半径
        cv.putText(Ileather, (str(centre_x) + "," + str(centre_y)), (centre_x - 50, centre_y + 30), font, 1,
                   (0, 255, 0), 2)

    # cv.imshow("output",Ileather)
    ###create coordination###
    cv.line(Ileather, (0, 0), (0 + 100, 0), (0, 255, 0), 2)
    cv.line(Ileather, (0, 0), (0, 0 + 100), (0, 255, 0), 2)
    cv.putText(Ileather, 'X axis', (120, 40), font, 2, (0, 255, 0), 2)
    cv.putText(Ileather, 'Y axis', (10, 130), font, 2, (0, 255, 0), 2)
    print('run here')
    cv.namedWindow("processed", cv.WINDOW_AUTOSIZE)
    cv.imshow("processed", Ileather)
    # cncdrawcircle(centre_x, centre_y)
    cv.waitKey(5000)
    cv.destroyWindow("processed")

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


# create machine vision threading-1
class myThread(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

    def run(self):
        time.sleep(5)
        capture = cv.VideoCapture(0)
        while 1:
            ret, self.frame = capture.read()

            cv.imshow("video", self.frame)
            c = cv.waitKey(1)


    def get_result(self):
        return self.frame


thread_vision = myThread(threadID=1, name="machinevision")
thread_vision.start()
while True:
    # todo:banzidong quanzidongqiehuan
    time.sleep(30)
    # multiple threadings

    # todo：action to cloud
    updateMsn = {
        # 'PowerLed':
        'semiautomatic': cncstart
    }
    JsonUpdataMsn = aliLink.Alink(updateMsn)
    print(JsonUpdataMsn)

    mqtt.push(POST, JsonUpdataMsn)  # 定时向阿里云IOT推送我们构建好的Alink协议数据
capture.release()
cv.destroyAllWindows()
