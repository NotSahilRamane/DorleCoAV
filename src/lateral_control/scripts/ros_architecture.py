#!/usr/bin/env python3

#import libraries
from audioop import avg
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from lateral_control import lateral_control
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from av_messages.msg import laneDetectionFlag , laneDetectionInfo

class Ros_architecture(object):
    def __init__(self):
        self.loadParameters()
        self.lka_lca=lateral_control()
        self.prv_dt = rospy.get_time()
        self.lateral_offset_error=0
        self.roadHeadingAngle=0
        self.FullBrakingDeceleration=5 #-5 m/s^2
        self.Prev_SteeringAngle=0
        self.ego_velocity_x = 0
        self.ego_velocity_y = 0
        self.ego_Heading = 0

    def loadParameters(self):
        self.laneDetectionFlag=rospy.get_param("lateral_control/lane_detection_flag","/lateral_control/laneDetectionFlag")
        self.laneDetectionData=rospy.get_param("lateral_control/lane_detection_info","/lateral_control/laneDetectionInfo")
        self.ego_veh_velocity=rospy.get_param("lateral_control/Ego_velocity","/Carla/ego_vehicle/Odometry")
        self.pub_topic_steering=rospy.get_param("lateral_control/Ego_Steering","/Ego_vehicle/controls/steering")
        # self.pub_LKA_braking_deceleartion=rospy.get_param("lateral_control/Ego_LKA_Braking","/Ego_vehicle/controls/Braking")
    def subscribeToTopics(self):
        rospy.loginfo("subscribing to the topics")
        rospy.Subscriber(self.laneDetectionFlag, laneDetectionFlag,self.LaneDetectionconfirmed,queue_size=1)
        rospy.Subscriber(self.laneDetectionData,laneDetectionInfo,self.laneDetectionPreprocessing,queue_size=1)
        rospy.Subscriber(self.ego_veh_velocity,Odometry,self.getSteering,queue_size=1)
        

    def publishToTopics(self):
        self.controlPublisherSteering=rospy.Publisher(self.pub_topic_steering,Twist,queue_size=1)
    
    def LaneDetectionconfirmed(self,laneDetecionFlag):
        if laneDetectionFlag==True:
            self.Calculate_lane_detection_info=True
        else:
            self.Calculate_lane_detection_info=False

            
    def laneDetectionPreprocessing(self, laneDetectionInfo):
        if self.Calculate_lane_detection_info==True:
            self.lateral_offset_error=laneDetectionInfo.lateralOffset
            self.roadHeadingAngle=laneDetectionInfo.RoadHeadingAngle
            self.calculate_steering=True
        else:
            self.calculate_steering=False


    def getSteering(self,Odometry):
        if self.calculate_steering==True:
            self.ego_velocity_x = Odometry.twist.twist.linear.x
            self.ego_velocity_y = Odometry.twist.twist.linear.y
            self.ego_Heading = Odometry.pose.pose.orientation.z
            BrakingDeceleration, Steering_Angle, SteeringAngleAVG=self.Action_loop()
        else:
            BrakingDeceleration, Steering_Angle, SteeringAngleAVG=self.Action_loop()
            BrakingDeceleartion= np.max(self.FullBrakingDeceleration,BrakingDeceleartion)
            Steering_Angle=SteeringAngleAVG

        control_message=Twist()
        control_message.linear.y=BrakingDeceleration
        control_message.linear.z=Steering_Angle
        self.call_publisher(control_message)
          
    def Action_loop(self):
        BrakingDeceleration, Steering_Angle=self.lka_lca()
        self.Prev_SteeringAngle=+Steering_Angle
        SteeringAngleAVG=self.Prev_SteeringAngle/2
        return BrakingDeceleration, Steering_Angle, SteeringAngleAVG

    def call_publisher(self,controls):
        self.controlPublisherSteering.publish(controls)
        print("Published the Steering Angle")