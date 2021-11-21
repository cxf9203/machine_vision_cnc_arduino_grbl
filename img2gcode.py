#__author:StevenChen
#__Date:2021/11/21
import cv2 as cv
import image_to_gcode as i2g
img = cv.imread("align2_test.bmp")
output = i2g.ImageToGcode(img)
