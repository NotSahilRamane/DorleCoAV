#!/usr/bin/env python3

import numpy as np
import rospy

from av_messages.msg import object
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleStatus 
from adas_features import AEB_Controller
# from geometry_msgs.msg import Accel

class ADAS_Features:
    def __init__(self):
        self.loadParameters()
        self.time_step = 0
        # self.bridge = CvBridge()
        self.aeb = AEB_Controller()


    def subscribeToTopics(self):
        rospy.loginfo('Subscribed to topics')
        rospy.Subscriber(self.MIO_object_topicname, object,
                        self.extractDataMIO, queue_size=1)
        rospy.Subscriber(self.road_seg_topicname, Twist,
                        self.extractDataRoadSeg, queue_size=1)
        rospy.Subscriber(self.ego_veh_velocity_topicname, Odometry,
                        self.extractEgoVehVelocity, queue_size=1)
        rospy.Subscriber(self.getAccel_topicname, CarlaEgoVehicleStatus,
                        self.getAccel, queue_size=1)

    def getAEBParams(self):
        v1 = min(self.distance, self.MIO_position)
        v2 = self.MIO_velocity
        v3 = 20                                                  # constant for now should change later 
        if v2 != 0:
            v4 = v1/v2
        else:
            v4 = inf
        v5 = self.acceleration
        v6 = driver_brake
        v7 = dt
        return v1, v2, v3, v4, v5, v6, v7

# MIO: MOST IMPORTANT OBJECT

    def loadParameters(self):
        self.MIO_object_topicname = rospy.get_param("road/mio", "/road/mio") 
        self.road_seg_topicname = rospy.get_param("road/lookahead", "/road/lookahead") 
        self.ego_veh_velocity_topicname = rospy.get_param("perception_adas_conn/ego_velocity","/carla/ego_vehicle/odometry")
        self.getAccel_topicname = rospy.get_param("perception_adas_conn/ego_accel", "/carla/ego_vehicle/vehicle_status")
        self.pub_topic_name = rospy.get_param("perception_adas_conn/adas_features_control", "/vehicle/controls") 
    
    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.ControlsPublisher = rospy.Publisher(
            self.pub_topic_name, Twist, queue_size=1)
        
    def extractDataMIO(self, obj_data):
        self.MIO_velocity = obj_data.object_state_dt
        self.MIO_position = obj_data.position

    def extractDataRoadSeg(self, seg_data):
        self.go_flag = seg_data.linear.x
        self.distance = seg_data.linear.y

    def extractEgoVehVelocity(self, odometry):
        self.ego_velocity_x = odometry.twist.linear.x # an extra twist might be req.
        self.ego_velocity_y = odometry.twist.linear.y
        self.ego_velocity_angular = odometry.angular.z

    def getAccel(self, acceleration):
        self.acceleration = acceleration.Accel.linear.x # check if Accel is necessary

    def Algorithm(self):
        relative_dist, ego_vel, ACC_set_speed, ttc, ego_acc, driver_brake, dt = self.getAEBParams()
        controls = self.aeb.get_controls()
        self.callPublisher(controls)

    def callPublisher(self, controls):
        self.ControlsPublisher.publish(controls)

# object - velocity, pos
# road-seg - geometry/twist
# vehicle_velot - odometry
# vehicle_accel - CarlaVehicleStatus/Accel
