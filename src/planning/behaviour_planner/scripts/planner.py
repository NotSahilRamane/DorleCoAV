#!/usr/bin/env python33

import rospy

import numpy as np
import time
import math
from ros_numpy import occupancy_grid

from av_messages.msg import carState, trafficLights, objects, destination, behaviour, globalPlan
from nav_msgs.msg import OccupancyGrid

class BehaviourPlanner:
    def __init__(self):
        self.loadParameters()

    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")
        rospy.Subscriber(self.global_plan_topic_name, globalPlan, buff_size=2**24, queue_size=1, callback=self.globalPlanCallback)
        rospy.Subscriber(self.current_state_topic_name, carState, callback=self.stateCallback)
        rospy.Subscriber(self.slam_map_topic_name, OccupancyGrid, buff_size=2**24, callback=self.slamCallback)
        rospy.Subscriber(self.objects_topic_name, objects, buff_size=2**24, callback=self.objectCallback)  
        rospy.Subscriber(self.destination_topic_name, destination, callback=self.destinationCallback)  
        rospy.Subscriber(self.traffic_lights_topic_name, trafficLights, callback=self.trafficLightCallback)    


    def loadParameters(self):
        '''
        add data
        '''

    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.behaviourPublisher = rospy.Publisher(
            self.pub_topic_name, behaviour, queue_size=1)

    def globalPlanCallback(self, data):
        self.globalPlan = data.waypoints

    def stateCallback(self, data):
        self.carPos = data.car_state
        self.carVel = data.car_state_dt

    def slamCallback(self, data):
        self.slam_map = occupancy_grid.occupancygrid_to_numpy(data)
        self.callPlanner()
    
    def destinationCallback(self, data):
        self.destination = data

    def trafficLightCallback(self, data):
        self.traffic_light_data = data

    def objectCallback(self, data):
        self.obstacle_data = data
        self.callPlanner()

    def callPlanner(self):
        '''
        Put Local Planner related data here
        '''

    def callPublisher(self):
        '''
        to be added
        '''