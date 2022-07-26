#!/usr/bin/env python3

import numpy as np
import rospy
import math

from av_messages.msg import object_
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleStatus 
from AEB_Controller import AEB_Controller
# from geometry_msgs.msg import Accel

class ADAS_Features:
    def __init__(self):
        self.loadParameters()
        self.time_step = 0
        # self.bridge = CvBridge()
        self.MIO_position = 0
        self.MIO_DATA_RECEIVED = 0
        self.ROAD_SEG_RECEIVED = 0
        self.aeb = AEB_Controller()
        self.prv_dt = rospy.get_time()


    def subscribeToTopics(self):
        rospy.loginfo('Subscribed to topics')
        rospy.Subscriber(self.MIO_object_topicname, object_,
                        self.extractDataMIO, queue_size=1)
        rospy.Subscriber(self.road_seg_topicname, Twist,
                        self.extractDataRoadSeg, queue_size=1)
        rospy.Subscriber(self.ego_veh_velocity_topicname, Odometry,
                        self.extractEgoVehVelocity, queue_size=1)
        rospy.Subscriber(self.getAccel_topicname, CarlaEgoVehicleStatus,
                        self.getAccel, queue_size=1)

    def getAEBParams(self):
        relative_dist = min(self.drivable_distance, self.MIO_position)
        relative_vel = self.MIO_velocity
        ACC_set_speed = 20                                                  # constant for now should change later 
        if relative_vel != 0:
            ttc = relative_dist/relative_vel
        else:
            ttc = float('inf')
        ego_acc = self.acceleration
        driver_brake = 0
        dt = rospy.get_time() - self.prv_dt # add time stamps
        self.prv_dt = rospy.get_time()
        return relative_dist, relative_dist, ACC_set_speed, ttc, ego_acc, driver_brake, dt

# MIO: MOST IMPORTANT OBJECT

    def loadParameters(self):
        self.MIO_object_topicname = rospy.get_param("road/mio", "/road/mio") 
        self.road_seg_topicname = rospy.get_param("road/deadahead", "/road/deadahead") 
        self.ego_veh_velocity_topicname = rospy.get_param("perception_adas_conn/ego_velocity","/carla/ego_vehicle/odometry")
        self.getAccel_topicname = rospy.get_param("perception_adas_conn/ego_accel", "/carla/ego_vehicle/vehicle_status")
        self.pub_topic_name = rospy.get_param("perception_adas_conn/adas_features_control", "/vehicle/controls") 

    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.ControlsPublisher = rospy.Publisher(
            self.pub_topic_name, Twist, queue_size=1)

    def extractDataMIO(self, obj_data):
        if self.MIO_DATA_RECEIVED == 0:
            self.MIO_velocity = obj_data.object_state_dt.y
            self.MIO_position = obj_data.position.y
            self.MIO_DATA_RECEIVED = 1
            print("MIO Filled")

            self.sync_data()

    def extractDataRoadSeg(self, seg_data):
        if self.ROAD_SEG_RECEIVED == 0:
            self.go_flag = seg_data.linear.x ## FLAG in linear.x
            self.drivable_distance = seg_data.linear.y
            self.ROAD_SEG_RECEIVED = 1
            print("Seg Filled")
            self.sync_data()

    def sync_data(self): # Copy for Obj Detection
        if self.ROAD_SEG_RECEIVED == 1 and self.MIO_DATA_RECEIVED == 1:
            self.Algorithm()
            self.MIO_DATA_RECEIVED = 0
            self.ROAD_SEG_RECEIVED = 0
        else:
            print("Either Flag is Zero")
            # if self.ROAD_SEG_RECEIVED == 1:
            #     self.ROAD_SEG_RECEIVED = 0
            
            # elif self.MIO_DATA_RECEIVED == 1:
            #     self.MIO_DATA_RECEIVED = 0
            # # self.MIO_DATA_RECEIVED = 0
            # self.ROAD_SEG_RECEIVED = 0

    def extractEgoVehVelocity(self, odometry):
        self.ego_velocity_x = odometry.twist.twist.linear.x
        self.ego_velocity_y = odometry.twist.twist.linear.y
        self.ego_velocity_angular = odometry.pose.pose.orientation.z
        # print("EGO VEL RECEIVED")

    def getAccel(self, acceleration):
        self.acceleration = acceleration.acceleration.linear.x
        # print("EGO ACC RECEIVED")

    def Algorithm(self):
        control_message = Twist()
        relative_dist, relative_vel, ACC_set_speed, ttc, ego_acc, driver_brake, dt = self.getAEBParams()
        brake, throttle = self.aeb.get_controls(relative_dist, relative_vel, ACC_set_speed, ttc, ego_acc, driver_brake, dt)
        # brake = self.stable_sigmoid(brake)
        # throttle = self.stable_sigmoid(throttle)
        # brake, throttle = brake/2, throttle/2
        brake = 1-math.exp(-1*brake)
        brake = max(0, min(1, brake))
        throttle = 1-math.exp(-1*throttle)
        throttle = max(0, min(1, throttle))
        print(brake, throttle)
        # Fill twist message here ###
        control_message.linear.x = throttle
        control_message.linear.y = brake
        self.callPublisher(control_message)

    def stable_sigmoid(self, x):

        if x >= 0:
            z = math.exp(-x)
            sig = 1 / (1 + z)
            return sig
        else:
            z = math.exp(x)
            sig = z / (1 + z)
            return sig

    def callPublisher(self, controls):
        self.ControlsPublisher.publish(controls)
        print("Published AEB Message")

# object - velocity, pos
# road-seg - geometry/twist
# vehicle_velot - odometry
# vehicle_accel - CarlaVehicleStatus/Accel
