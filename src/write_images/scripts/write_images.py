#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

# ROS packages
import roslib
import rospy
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Int16
roslib.load_manifest('write_images')

# openCV package
import cv2
from cv_bridge import CvBridge, CvBridgeError

import sys
import numpy as np

class image_converter:

    def __init__(self):
        # state variables
        self.steering = 0
        self.speed = 0
        self.record_button_state = 0
        self.image_to_write = None
        self.current_sim_time = None

        self.bridge = CvBridge()
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.read_joystick_callback)
        self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.image_callback, queue_size=10)
        self.steering_sub = rospy.Subscriber("/AutoNOMOS_mini/manual_control/steering",Int16,self.steering_callback, queue_size=10)
        self.speed_sub = rospy.Subscriber("/AutoNOMOS_mini/manual_control/speed",Int16,self.speed_callback, queue_size=10)

    def image_callback(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
          print(e)

        #gray
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
        mask_roi = mask_image[240:410,:] # 170,640

        # resize
        mask_roi = cv2.resize(mask_roi,(80,40))

        # get current time
        self.current_sim_time = data.header.stamp
        # offset
        self.current_sim_time = int(str(self.current_sim_time)) + 2146201000000

        # lanes to write one hot encoding
        out_left =  '0'
        left =      '1'
        center =    '0'
        right =     '0'
        out_right = '0'

        if self.record_button_state:
            # write images
            cv2.imwrite(str(self.current_sim_time) + '.png', mask_roi)


            # write labels 
            with open('labels.txt', 'a') as myfile:
                myfile.write(str(self.steering) + ', ' + out_left
                                                + ', ' + left 
                                                + ', ' + center
                                                + ', ' + right
                                                + ', ' + out_right
                                                + ', ' + str(self.current_sim_time) + '\n')

    def steering_callback(self,data):
        self.steering = data.data

    def speed_callback(self,data):
        self.speed = data.data

    def read_joystick_callback(self, joystick_input):
        # record/pause
        if (joystick_input.buttons[5] == 1):
            self.record_button_state = 1
            print('RECORDING...')
        if (joystick_input.buttons[5] == 0):
            self.record_button_state = 0
            print('...')


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
