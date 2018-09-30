#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

# ROS packages
import roslib
import rospy
from sensor_msgs.msg import Image
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
import matplotlib.pyplot as plt
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
        self.msg_timestamps = []
        self.running_framerate = None
        self.network = None
        self.image_received = None

        self.speed = 0
        self.steering = 90

        # cnn settings
        self.settings = Settings()
        self.bridge = CvBridge()

        self.settings.ydim = 40
        self.settings.xdim = 80
        self.settings.channels = 1
        self.settings.model_weights = "/DATA/Documents/ai_controlled_car/src/ai_car_control/weights/v2.h5"

        # load model
        print("WARMING UP...")
        self.network = CNN(settings=self.settings)
        self.model = self.network.build_model(lr=1e-3)
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()
        print('MODEL BUILDED...')


        # Communication

        # real car
        # self.steering_pub = rospy.Publisher("/manual_control/steering", Int16, queue_size=10)
        # self.speed_pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=10)

        # Simulation
        self.image_sub = rospy.Subscriber("/image_preprocessed",Image,self.image_callback, queue_size=10)
        self.steering_pub = rospy.Publisher("/AutoNOMOS_mini/manual_control/steering", Int16, queue_size=10)
        self.speed_pub = rospy.Publisher("/AutoNOMOS_mini/manual_control/speed", Int16, queue_size=10)



    def image_callback(self,data):
        try:
          self.image_received = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono8')
          self.image_received = self.image_received.reshape(np.append(1,self.image_received.shape))
          self.image_received = self.image_received.reshape(np.append(self.image_received.shape,1))
          self.predict(self.image_received)
        except CvBridgeError as e:
          print(e)


    def predict(self, image):
        image = image/255

        # acceleration
        self.speed = -500
        self.pubSpeed(self.speed)
        # steering

        self.steering = self.model.predict(image)
        self.steering = self.steering * 180
        self.pubSteering(self.steering)
        print('steering --> ' + str(self.steering))



    def stop(self):
        self.pubSpeed(0)

    def pubSteering(self, deg_value):
        self.steering_pub.publish(deg_value)

    def pubSpeed(self, speed):
        self.speed_pub.publish(speed)

    def print_state(self):
        print("Speed    --> %d " %(self.speed))
        print("Steering --> %d " %(self.steering))

def main(args):
  rospy.init_node('ai_car_control', anonymous=True)
  car = CarController()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
