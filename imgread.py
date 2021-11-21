import cv2
import numpy as np
from matplotlib import pyplot as plt


img = cv2.imread('aisha.jpg')
blur = cv2.blur(img,(3,3))

cv2.imshow("winName",blur)
cv2.waitKey(0)


