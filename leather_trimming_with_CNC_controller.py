'''
最小外接圆方法，尝试使用最小矩形做切割
方法一：利用模板找到合适的圆

'''
import aliLink,mqttd
import paho.mqtt.client
import time,json,random
import hmac,hashlib
import wget
import os
import zipfile
import cv2 as cv
import numpy as np
import math
import serial

#serial setting
serialPort="com3"
baudRate = 9600
ser = serial.Serial(serialPort,baudRate,timeout=0.5)

#CNC G code programming
def cncdrawcircle(x,y):
    gcodecmd = "G01 "+"X"+str(x) +" Y"+str(y)
    print("gcode cmd is :",gcodecmd)
    #todo:
    #need a patten to milling
    ser.write((gcodecmd + '\r\n').encode())

# 消息回调（云端下发消息的回调函数）
def on_message(client, userdata, msg):
    print(msg.payload)
    #print(msg.payload)  # 开关值


# 连接回调（与阿里云建立链接后的回调函数）
def on_connect(client, userdata, flags, rc):
    pass

###computer vision settings###
leatherImage = "leather1.jpg"
CircleRadius = 100 #try other number: 50 200 etc..
forSteps = 20
areaTolerance = 5
radiusTolerance = 2
mat = np.float32([[1,0,forSteps],[0,1,forSteps]])
contouredge = 5 #segment circles

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






###read leather image###
Ileather = cv.imread(leatherImage)#input image
#Ileather =cv.resize(Ileather,None,fx=0.5,fy=0.5,interpolation=cv.INTER_CUBIC)
#img =Ileather.copy()

#cv.imshow("original",Ileather)
#img_blue = img[:,:,0]
#cv.imshow("blue",img_blue)
#img_green = img[:,:,1]
#cv.imshow("green",img_green)
#img_red = img[:,:,2]
#cv.imshow("red",img_red)

###change to grayscale###
Ileather_gray = cv.cvtColor(Ileather,cv.COLOR_RGB2GRAY) #image to gray
#cv.imshow("original_gray",Ileather_gray)

###change to black and white(binary)###
ret, Ileather_binary = cv.threshold(Ileather_gray,127,255,cv.THRESH_BINARY)
#cv.imshow("binary",Ileather_binary)



###create Odd Row Circles###
size = Ileather_gray.shape
#print(size)
m = size[0] #hight rows
n = size[1] #width column
#print(size)
#print(m)
#print(n)

"""get template"""
#### Create Odd Row Circles###
CentreX = CircleRadius
CentreXArrayOdd = [CentreX]
for i in range(n):
    CentreX = CentreX + 2 * CircleRadius+contouredge # plus 1 for contour
    if CentreX > (n-CircleRadius):
        break
    CentreXArrayOdd.append(CentreX) #修改数组并添加数
#print(CentreXArrayOdd)


###create even row Circles###
CentreX = 2*CircleRadius
CentreXArrayEven = [CentreX]
for i in range(n): ##column
    CentreX = CentreX + 2*CircleRadius+contouredge      # plus 1 for contour
    if CentreX > (n - CircleRadius):
        break
    CentreXArrayEven.append(CentreX)  #修改数组并添加数
#print(CentreXArrayEven)


###Create Odd Column Circles###
CentreY = CircleRadius
CentreYArrayOdd = [CentreY]

for i in range(m): #####row
    CentreY = CentreY + math.ceil(contouredge+2*CircleRadius*0.87*2)  # plus 1 for contour #math.sin(60) = 0.866!!!!!!60度
    if CentreY > m - CircleRadius:
        break;
    CentreYArrayOdd.append(CentreY)
#print(CentreYArrayOdd)

####Create Even Column Circles###
CentreY =  CircleRadius+ math.ceil(2*CircleRadius*0.87)# plus 1 for contour #math.sin(60) = 0.866!!!!!! 60度
CentreYArrayEven = [CentreY]
for i in range(m): ###row
    CentreY = CentreY + math.ceil(contouredge+ 2*CircleRadius*0.87*2)#plus 1 for contour
    if CentreY > m - CircleRadius:
        break
    CentreYArrayEven.append(CentreY)
#print(CentreYArrayEven)


###Create Circles on the Centres###
CentreOdd = []
for i in range(len(CentreYArrayOdd)):
    for j in range(len(CentreXArrayOdd)):
        CentreOdd.append(( CentreXArrayOdd[j],CentreYArrayOdd[i]))
#print(CentreOdd)

CentreEven = []
for i in range(len(CentreYArrayEven)):
    for j in range(len(CentreXArrayEven)):
        CentreEven.append((CentreXArrayEven[j],CentreYArrayEven[i]))
#print(CentreEven)

###get Centre coordination in digital image###
Centre = CentreOdd + CentreEven  #get centre data
#Centre.append(CentreEven)  error
#print(Centre)
#print(len(Centre))
#get a black image
img_black = np.zeros((m,n,3),np.uint8)

size_img_black = img_black.shape
#print(size_img_black)
#cv.line(img_black,(0,0),(n,m),(255,0,0),5)

for k in range(len(Centre)):
    cv.circle(img_black,
              Centre[k],
              CircleRadius,
              (0,0,255),
              -1) ### -1>>the circle is filled with color
#cv.imshow("image_black",img_black)
###change to grayscale###
img_black_grayscale = cv.cvtColor(img_black,cv.COLOR_RGB2GRAY)

###change to black and white(binary)###
ret, img_black_binary = cv.threshold(img_black_grayscale,10,255,cv.THRESH_BINARY)
size_binary = img_black_binary.shape
#print(size_binary)
cv.imshow("black_binary",img_black_binary)
###calculate circle area###


######multiply binary####
#fist step： Search for optimal placement of circle using for loop
#这里有点问题，需要搞搞
#todo:
#need translation????
count = []
#for i in range(1,forSteps,2*CircleRadius):
    #for j in range(1,forSteps,2*CircleRadius):
##translate circle
img_black_binary = cv.warpAffine(img_black_binary, mat, (img_black_binary.shape[1], img_black_binary.shape[0]))
#cv.imshow("translatecircle",img_black_binary)
LeatherMultiplyCircles = cv.bitwise_and(img_black_binary, Ileather_binary, dst=None, mask=None)
#cv.imshow("LeatherMultiplyCircles",LeatherMultiplyCircles)

kernel = np.ones((3,3),int)
LeatherMultiplyCircles = cv.erode(LeatherMultiplyCircles,kernel) #腐蚀进来缩小边界接触点
#cv.imshow("LeatherMultiplyCircles2",LeatherMultiplyCircles)


###Hough圆检测方法###
contours,hierarchy = cv.findContours(LeatherMultiplyCircles,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE) #找到该对象外部轮廓
minRect = [None]*len(contours)
for i,c in enumerate(contours):
    minRect[i] = cv.minAreaRect(c)
#print("minimum rectangle area",minRect)

for i,c in enumerate(contours):
    box = cv.boxPoints(minRect[i])
    box = np.intp(box)

    #print('the box point lists',box)
    #if i==0:
     #   print('box[i, 0]',box[0])
    box_w = minRect[i][1][0] #get box2D structure,(x,y) and (w,h) from minRect
    box_h = minRect[i][1][1]
    #w = np.sqrt(np.sum(box_w ** 2))
    #print('w=', box_w)
    #h = np.sqrt(np.sum(box_h ** 2))
    #print('h=', box_h)
    if box_w*box_h>30000:
        if box_w<box_h+0.1 and box_w>box_h-0.1:
            cv.drawContours(Ileather, contours, i , (0, 0, 255),-1)
            #cv.drawContours(Ileather_binary, contours, i, 0, -1)
            M = cv.moments(c)
            #cv.imshow("LeatherMultiplyCirclesprocess", Ileather_binary)
            #print(M)
            centre_x, centre_y = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
            #print('circle centre x is :', centre_x, 'circle centre y is :', centre_y)
            cv.circle(Ileather,(centre_x,centre_y),7, (0,255,0), -1)
            cv.line(Ileather,(centre_x,centre_y),(centre_x+CircleRadius,centre_y),(0,255,0),2)
            cncdrawcircle(centre_x,centre_y)
    else:
        pass

###create coordination###
cv.line(Ileather,(0,0),(0+100,0),(0,255,0),2)
cv.line(Ileather,(0,0),(0,0+100),(0,255,0),2)
font=cv.FONT_HERSHEY_SIMPLEX
cv.putText(Ileather,'X axis',(120,40), font, 2,(0,255,0),2)
cv.putText(Ileather,'Y axis',(10,130), font, 2,(0,255,0),2)
cv.imshow("LeatherMultiplyCircles3",Ileather)


cv.waitKey(0)
cv.destroyAllWindows()

'''
备选方案
####圆检测 ####
###Hough圆检测方法###
circles = cv.HoughCircles(img_black, cv.HOUGH_GRADIENT, 1, CircleRadius,
                       param1=50, param2=30,
                       minRadius=CircleRadius-radiusTolerance, maxRadius=CircleRadius+radiusTolerance)
#print(circles)
circles_count =circles.shape[1]
print(circles)
###寻找最多圆的方案###
#draw circle
#print("circle count",circles_count)
if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0, :]:
        center = (i[0], i[1])
        # circle center
        cv.circle(Ileather, center, 1, (0, 100, 100), 3)
        # circle outline
        radius = i[2]
        cv.circle(Ileather, center, radius, (255, 0, 255), 3)
cv.imshow("hough circle",Ileather)
'''



