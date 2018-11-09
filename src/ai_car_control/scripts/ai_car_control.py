#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

# ROS packages
import roslib
import rospy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String, Int16
roslib.load_manifest('ai_car_control')

# keras
import keras
from keras.callbacks import ModelCheckpoint, LambdaCallback, TensorBoard
from keras.optimizers import RMSprop, SGD

# openCV package
import cv2
from cv_bridge import CvBridge, CvBridgeError

# others
import argparse
import os
from os import path
import time
from scipy import misc

from utils import Settings
from utils import DataLoader
from model_dense import CNN
import time
import sys
import numpy as np
import tensorflow as tf

class CarController:
    def __init__(self):
        # set-up
        self.network = None
        self.image_received = None
        self.lidar_received = None

        # parametric variables
        self.max_speed = rospy.get_param('~max_speed', '1000')

        # state variables
        self.speed = 0
        self.steering = 90
        self.out_left = 0
        self.left = 0
        self.center = 0
        self.right = 0
        self.out_right = 0

        # cnn settings
        self.settings = Settings()
        self.bridge = CvBridge()

        self.settings.ydim = 40
        self.settings.xdim = 80
        self.settings.channels = 1

        weights_path = rospy.get_param('~weights_path')
        self.settings.model_weights = weights_path

        # load model
        print("WARMING UP...")
        self.network = CNN(settings=self.settings)
        self.model = self.network.build_model(lr=1e-3)
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()
        print('MODEL BUILDED...')


        # Communication

        # real car
        self.steering_pub = rospy.Publisher("/manual_control/steering", Int16, queue_size=10)
        self.speed_pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=10)

        # Simulation
        car_ID = rospy.get_param('~car_ID')
        self.image_sub = rospy.Subscriber("/image_preprocessed", Image, self.image_callback, queue_size=10)
        self.image_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=10)
        # self.steering_pub = rospy.Publisher("/" + car_ID + "/manual_control/steering", Int16, queue_size=10)
        # self.speed_pub = rospy.Publisher("/" + car_ID + "/manual_control/speed", Int16, queue_size=10)



    def image_callback(self,data):
        try:
          self.image_received = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono8')
          self.image_received = self.image_received.reshape(np.append(1,self.image_received.shape))
          self.image_received = self.image_received.reshape(np.append(self.image_received.shape,1))
          self.predict(self.image_received)
        except CvBridgeError as e:
          print(e)

    def lidar_callback(self,data):
        self.lidar_received = data
        self.obstacle_avoidance()

    def predict(self, image):
        image = image/255
        prediction = self.model.predict(image)
        [self.steering, self.out_left, self.left, self.center, self.right, self.out_right] = prediction[0]
        self.steering = self.steering * 180
        # dynamic acceleration
        self.dynamic_speed()
        # publish speed
        self.pub_speed(self.speed)
        # obstacle avoidance implemented
        self.obstacle_avoidance()
        # print actual state variables
        self.print_state()

    def dynamic_speed(self):
        aux = self.speed
        # normalize
        if(abs(90 - self.steering) < 45):
            aux = abs(90 - self.steering)/45
        else:
            aux = 1
        #cuadratic interpolation
        aux = (1 - .375 * aux**2) * self.max_speed
        self.speed = -aux

    def stop(self):
        self.pub_speed(0)

    def pub_steering(self, deg_value):
        self.steering_pub.publish(deg_value)

    def pub_speed(self, speed):
        self.speed_pub.publish(speed)

    def obstacle_avoidance(self):
        ranges = np.array(self.lidar_received.ranges)
        min_distance = 1
        if (min(ranges[:8]) < min_distance or min(ranges[-14:-1]) < min_distance):
            self.pub_steering(180)
        elif (min(ranges[-160:-155]) < min_distance):
            self.pub_steering(0)
        else:
            self.pub_steering(self.steering)

    def print_state(self):
        print('steering     --> ' + str(self.steering))
        print('speed        --> ' + str(self.speed))
        print('out_left     --> ' + str(abs(self.out_left)))
        print('left         --> ' + str(abs(self.left)))
        print('center       --> ' + str(abs(self.center)))
        print('right        --> ' + str(abs(self.right)))
        print('out_right    --> ' + str(abs(self.out_right)))

def main(args):
  rospy.init_node('ai_car_control', anonymous=True)
  car = CarController()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
