#!/usr/bin/env python3

import rospy

import numpy as np
import time
import math
from ros_numpy import occupancy_grid

from av_messages.msg import carState, globalPlan, behaviour, localPlan, wayPoint
from nav_msgs.msg import OccupancyGrid

class LocalPlanner:
    def __init__(self):
        self.loadParameters()

    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")
        rospy.Subscriber(self.global_plan_topic_name, globalPlan, buff_size=2**24, queue_size=1, callback=self.globalPlanCallback)
        rospy.Subscriber(self.current_state_topic_name, carState, callback=self.stateCallback)
        rospy.Subscriber(self.slam_map_topic_name, OccupancyGrid, buff_size=2**24, callback=self.slamCallback)
        rospy.Subscriber(self.behaviour_topic_name, behaviour, callback=self.behaviourCallback)

    def loadParameters(self):
        '''
        add data
        '''

    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.waypointsPublisher = rospy.Publisher(
            self.pub_topic_name, localPlan, queue_size=1)

    def globalPlanCallback(self, data):
        self.globalPlan = data.waypoints

    def stateCallback(self, data):
        self.carPos = data.car_state
        self.carVel = data.car_state_dt

    def slamCallback(self, data):
        self.slam_map = occupancy_grid.occupancygrid_to_numpy(data)

    def behaviourCallback(self, data):
        self.behaviourData = data.behaviour

    def callPlanner(self):
        '''
        Put Local Planner related data here
        '''

    def callPublisher(self):
        '''
        to be added
        '''