import cv2
import numpy as np


winName='Colors of orange'
img_original = cv2.imread("orange.jpg")
img_hsv=cv2.cvtColor(img_original,cv2.COLOR_BGR2HSV)
#新建窗口


cv2.namedWindow(winName)
cv2.resizeWindow(winName, 720, 680)
lowerbH = 9
lowerbS = 85
lowerbV = 168
upperbH = 154
upperbS = 211
upperbV = 255

img_target=cv2.inRange(img_original,(lowerbH,lowerbS,lowerbV),(upperbH,upperbS,upperbV))
#输入图像与输入图像在掩模条件下按位与，得到掩模范围内的原图像
img_specifiedColor=cv2.bitwise_and(img_original,img_original,mask=img_target)



#erode
kernel = np.ones((5,5),np.uint8)
#开运算降噪
opening = cv2.morphologyEx(img_specifiedColor, cv2.MORPH_OPEN, kernel)
#闭运算去除前景物体小黑点
closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
#轮廓寻找
imbgr = cv2.cvtColor(closing,cv2.COLOR_HSV2BGR)
imgray = cv2.cvtColor(imbgr,cv2.COLOR_BGR2GRAY)
ret,thresh = cv2.threshold(imgray,0,255,0)
contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


#img = cv2.drawContours(img_original, contours,-1, (0,255,0), 3)

cnt = contours[0]
print(cnt)
area = cv2.contourArea(cnt)

x,y,w,h = cv2.boundingRect(cnt)
aspect_ratio = float(w)/h
print(aspect_ratio)
err = aspect_ratio-1
print(err)
abserr = abs(err)
if abserr < 0.5:
    insertrectange = cv2.rectangle(img_original, (x, y), (x + w, y + h), (0, 255, 0), 2)
else:
    insertrectange = img_original







#cv2.imshow("inshow",insertrectange)
M = cv2.moments(cnt)
print(M)

#cv2.imshow(winName,img)
#cv2.waitKey(0)





cap = cv2.VideoCapture(0)
while (True):
    ret, frame = cap.read()   #读取视频设备并读取视频帧    frame = img_original
    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    img_target = cv2.inRange(frame, (lowerbH, lowerbS, lowerbV), (upperbH, upperbS, upperbV))
    # 输入图像与输入图像在掩模条件下按位与，得到掩模范围内的原图像
    img_specifiedColor = cv2.bitwise_and(frame, frame, mask=img_target)
    # 开运算降噪
    opening = cv2.morphologyEx(img_specifiedColor, cv2.MORPH_OPEN, kernel)
    # 闭运算去除前景物体小黑点
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    # 轮廓寻找
    imbgr = cv2.cvtColor(closing, cv2.COLOR_HSV2BGR)
    imgray = cv2.cvtColor(imbgr, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 0, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #img = cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
    try:
        cnt = contours[0]
        x, y, w, h = cv2.boundingRect(cnt)
        err = float(w) / h - 1

        abserr = abs(err)
        area = cv2.contourArea(cnt)

        if abserr < 0.5 and area >150:
            frame1 = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        else:
            frame1 = frame
    except:
        continue




    cv2.imshow("ing",frame1)
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()



"""
    x, y, w, h = cv2.boundingRect(cnt)
    err = float(w) / h- 1

    abserr = abs(err)
    if abserr < 0.5:
        insertrectange = cv2.rectangle(img_original, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.imshow("insert", insertrectange)

"""


