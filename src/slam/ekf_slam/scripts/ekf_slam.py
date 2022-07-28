#!/usr/bin/env python33

import rospy

import numpy as np
import time

from geometry_msgs.msg import TwistStamped, PolygonStamped, Point32
from sensor_msgs.msg import Imu
from av_messages.msg import objects
from nav_msgs.msg import OccupancyGrid


import math

class EKFSlam:
    def __init__(self):
        self.loadParameters()

    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")
        rospy.Subscriber(self.obstacle_detections_topic_name, objects, buff_size=2**24, queue_size=1, callback=self.slamCallback)
        rospy.Subscriber(self.velocity_topic_name, TwistStamped, callback=self.velocityCallback)
        rospy.Subscriber(self.imu_topic_name, Imu, callback=self.yawrateCallback)

    def loadParameters(self):
        '''
        add data
        '''

    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.MapPublisher = rospy.Publisher(
            self.pub_topic_name, OccupancyGrid, queue_size=1)

    def slamCallback(self):
        '''
        add slam related code here
        '''

    def publishMap(self):
        '''
        to be added by Sahil
        '''

    def velocityCallback(self, data):
        x_vel = data.twist.linear.x
        y_vel = data.twist.linear.y
        self.velocity = np.sqrt(x_vel ** 2 + y_vel ** 2)

    def yawrateCallback(self, data):
        self.yawrate = data.angular_velocity.z
        
    