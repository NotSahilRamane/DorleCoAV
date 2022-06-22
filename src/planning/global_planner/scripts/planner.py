#!/usr/bin/env python3

import rospy

import numpy as np
import time
import math
from ros_numpy import occupancy_grid

from av_messages.msg import carState, globalPlan, behaviour, localPlan, wayPoint, destination
from nav_msgs.msg import OccupancyGrid
from graph_msgs.msg import GeometryGraph

class GlobalPlanner:
    def __init__(self):
        self.loadParameters()
    
    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")
        rospy.Subscriber(self.destination_topic_name, destination, buff_size=2**24, queue_size=1, callback=self.destinationCallback)
        rospy.Subscriber(self.current_state_topic_name, carState, callback=self.stateCallback)
        rospy.Subscriber(self.initial_state_topic_name, carState, callback=self.initialStateCallback)
        rospy.Subscriber(self.global_map_topic_name, GeometryGraph, callback=self.graphMapCallback)

    def loadParameters(self):
        '''
        add data
        '''

    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.waypointsPublisher = rospy.Publisher(
            self.pub_topic_name, globalPlan, queue_size=1)

    def stateCallback(self, data):
        self.carPos = data.car_state
        self.carVel = data.car_state_dt

    def initialStateCallback(self, data):
        self.initialCarPos = data.car_state
        self.initialCarVel = data.car_state_dt

    def destinationCallback(self, data):
        self.destination = (data.latitude, data.longitude)
    
    def graphMapCallback(self, data):
        self.map_intersections = data.nodes
        self.map_roads = data.edges
        self.callPlanner()

    def callPlanner(self):
        '''
        Call the global planner algorithm here
        '''

    def callPublisher(self):
        '''
        Call the final publisher here
        '''
