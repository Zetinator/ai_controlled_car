#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

# ROS packages
import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
roslib.load_manifest('preprocessing')

# openCV package
import cv2
from cv_bridge import CvBridge, CvBridgeError

import sys
import numpy as np

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_preprocessed",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=10)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    except CvBridgeError as e:
      print(e)

    # change to hsv colorspace
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # apply color masks
    lower_white = np.array([0,0,0], dtype=np.uint8)
    upper_white = np.array([0,0,255], dtype=np.uint8)
    # lower_yellow = np.array([20,100,100], dtype=np.uint8)
    # upper_yellow = np.array([30,255,255], dtype=np.uint8)

    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    # mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
    mask_image = cv2.bitwise_and(gray, mask_white)

    # noise filters
    gauss_gray = cv2.GaussianBlur(mask_image,(5,5),0)

    # canny filter
    threshold1 = 50
    threshold2 = 150
    canny_edges = cv2.Canny(gauss_gray,threshold1,threshold2)

    # # ROI
    # mask = np.zeros_like(cv_image)


    cv2.imshow("Image_captured", canny_edges)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8'))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

