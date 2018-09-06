from __future__ import print_function
from __future__ import division

# openCV package
import cv2

import sys
import numpy as np
from matplotlib import pyplot as plt

# read test images
# cv_image = cv2.imread('/home/erickzetinator/Documents/test/src/preprocessing/resources/images/image_1489693452225589534.jpg',1)
# cv_image = cv2.imread('/home/erickzetinator/Documents/test/src/preprocessing/resources/images/image_1489693458501756701.jpg',1)
# cv_image = cv2.imread('/home/erickzetinator/Documents/test/src/preprocessing/resources/images/image_1489693463556760369.jpg',1)
# cv_image = cv2.imread('/home/erickzetinator/Documents/test/src/preprocessing/resources/images/image_1489693484085976496.jpg',1)
# cv_image = cv2.imread('/home/erickzetinator/Documents/test/src/preprocessing/resources/images/image_1489693488850864497.jpg',1)
# cv_image = cv2.imread('/home/erickzetinator/Documents/test/src/preprocessing/resources/images/image_1489693481897352079.jpg',1)
# cv_image = cv2.imread('/home/erickzetinator/Documents/test/src/preprocessing/resources/images/image_1489693519137697292.jpg',1)

gray =  cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

# noise filters
gray = cv2.GaussianBlur(gray,(7,7),0)
# remap to 0-255
gray = (gray/gray.max())*255
gray = gray.astype('uint8')

# mask_white = cv2.inRange(gray, 230, 255)
mask_white = cv2.inRange(gray, 200, 255)
mask_image = cv2.bitwise_and(gray, mask_white)

# ROI
mask_roi = np.zeros_like(gray)
mask_roi[240:410,:] = 1
mask_image = cv2.bitwise_and(mask_white, mask_roi)


plt.subplot(121)
plt.imshow(gray)
plt.subplot(122)
plt.imshow(mask_image)

plt.show()
