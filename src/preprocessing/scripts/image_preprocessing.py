#!/usr/bin/env python

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

    (rows,cols,channels) = cv_image.shape
    cv2.circle(cv_image, (rows/2,cols/2), 10, 255, thickness=1)

    cv2.imshow("Image_captured", cv_image)
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
