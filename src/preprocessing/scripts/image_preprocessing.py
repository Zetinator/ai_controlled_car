#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

# ROS packages
import roslib
import rospy
from sensor_msgs.msg import Image
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

    #gray
    gray =  cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # ROI
    mask_roi = gray[240:410,:] # 170,640

    # noise filters
    gray = cv2.GaussianBlur(gray,(7,7),0)
    # remap to 0-255
    gray = (gray/gray.max())*255
    gray = gray.astype('uint8')

    # mask_white = cv2.inRange(gray, 230, 255)
    mask_white = cv2.inRange(gray, 200, 255)
    mask_image = cv2.bitwise_and(gray, mask_white)


    # resize
    image_out = cv2.resize(mask_image,(80, 40))

    # cv2.imshow('original... ', cv_image)
    # cv2.imshow('show me the goodies... ', mask_roi)
    # cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_out, encoding='mono8'))
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
