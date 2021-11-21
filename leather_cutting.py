import cv2 as cv
import numpy as np
import math

###settings###
leatherImage = "leather1.jpg"
CircleRadius = 100 #try other number: 50 200 etc..
forSteps = 20
areaTolerance = 5
radiusTolerance = 4
mat = np.float32([[1,0,forSteps],[0,1,forSteps]])
contouredge = 5 #segment circles

###read leather image###
Ileather = cv.imread(leatherImage)
#Ileather =cv.resize(Ileather,None,fx=0.5,fy=0.5,interpolation=cv.INTER_CUBIC)
img =Ileather.copy()

#cv.imshow("original",Ileather)
img_blue = img[:,:,0]
#cv.imshow("blue",img_blue)
img_green = img[:,:,1]
#cv.imshow("green",img_green)
img_red = img[:,:,2]
#cv.imshow("red",img_red)

###change to grayscale###
Ileather_gray = cv.cvtColor(Ileather,cv.COLOR_RGB2GRAY)
cv.imshow("original_gray",Ileather_gray)

###change to black and white(binary)###
ret, Ileather_binary = cv.threshold(Ileather_gray,127,255,cv.THRESH_BINARY)
cv.imshow("binary",Ileather_binary)



###create Odd Row Circles###
size = Ileather_gray.shape
#print(size)
m = size[0] #hight rows
n = size[1] #width column
print(size)
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
print(size_img_black)
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
count = []
for i in range(1,forSteps,2*CircleRadius):
    for j in range(1,forSteps,2*CircleRadius):
        ##translate circle
        img_black_binary = cv.warpAffine(img_black_binary, mat, (img_black_binary.shape[1], img_black_binary.shape[0]))
        #cv.imshow("translatecircle",img_black_binary)
        LeatherMultiplyCircles = cv.bitwise_and(img_black_binary, Ileather_binary, dst=None, mask=None)
        cv.imshow("LeatherMultiplyCircles",LeatherMultiplyCircles)


        multi_countours,hierarchy = cv.findContours(LeatherMultiplyCircles,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE) #找到该对象外部轮廓

        multi_img = cv.drawContours(Ileather, multi_countours, -1, (0,0,255), 1)
        multi_img_grayscale = cv.cvtColor(multi_img,cv.COLOR_RGB2GRAY)   #画出外部轮廓
        cv.imshow("multiimg",multi_img_grayscale)


####圆检测 #### -子轩部分
###to do->


        ###Hough圆检测方法###
        circles = cv.HoughCircles(multi_img_grayscale, cv.HOUGH_GRADIENT, 1, CircleRadius,
                               param1=100, param2=30,
                               minRadius=CircleRadius-radiusTolerance, maxRadius=CircleRadius+radiusTolerance)
        #print(circles)
        circles_count =circles.shape[1]
        print(circles)



###寻找最多圆的方案###


#draw circle
print("circle count",circles_count)
if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0, :]:

        center = (i[0], i[1])
        # circle center
        cv.circle(Ileather, center, 1, (0, 100, 100), 3)
        # circle outline
        radius = i[2]
        if radius > 99.5:
            cv.circle(Ileather, center, radius, (255, 0, 255), 3)
cv.imshow("hough circle",Ileather)
cv.waitKey(0)
cv.destroyAllWindows()
