#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

# ROS packages
import roslib
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String,Int16
roslib.load_manifest('car_control')

import time
import sys
import numpy as np


class CarController:
    def __init__(self):
        self.msg_timestamps = []
        self.running_framerate = None

        self.speed = 0
        self.steering = 90

        # Communication

        # real car
        # self.steering_pub = rospy.Publisher("/manual_control/steering", Int16, queue_size=10)
        # self.speed_pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=10)

        # Simulation
        self.steering_pub = rospy.Publisher("/AutoNOMOS_mini/manual_control/steering", Int16, queue_size=10)
        self.speed_pub = rospy.Publisher("/AutoNOMOS_mini/manual_control/speed", Int16, queue_size=10)


        rospy.topics.Subscriber("/joy", Joy, self.read_joystick)

        # ai
        # rospy.topics.Subscriber("/manual_control/speed", Int16, self.setSpeed)
        # rospy.topics.Subscriber("/manual_control/steering", Int16, self.setSteering)



    def read_joystick(self, joystick_input):
        # manual brake
        if (joystick_input.buttons[0] != 0):
            self.stop()
        # acceleration
        if (joystick_input.axes[5] < 1):
            self.pubSpeed((joystick_input.axes[5]-1)/2 *280)
        # reverse
        if (joystick_input.buttons[4] and joystick_input.axes[2] < 1):
            self.pubSpeed((joystick_input.axes[5]-1)/2 * (-280))
        # steering
        self.pubSteering(joystick_input.axes[0]*(90) + 90)


    def stop(self):
        self.pubSpeed(0)

    # TODO: ai_control
    # def setSpeed(self, speed):
    #     self.speed = speed.data
    #
    # def setSteering(self, steering):
    #     self.steering = steering.data

    def pubSteering(self, deg_value):
        self.steering_pub.publish(deg_value)

    def pubSpeed(self, speed):
        self.speed_pub.publish(speed)

    def print_state(self):
        print("Speed    --> %d " %(self.speed))
        print("Steering --> %d " %(self.steering))

def main(args):
  rospy.init_node('car_control', anonymous=True)
  car = CarController()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
