#!/usr/bin/env python3

import numpy as np
import rospy
import math

from av_messages.msg import fcw_perception
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleStatus 
from AEB_Controller import AEB_Controller
from geometry_msgs.msg import Twist
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
        rospy.Subscriber(self.perception_data_topic_name, fcw_perception,
                        self.extractDataMIO, queue_size=1)
        rospy.Subscriber(self.ego_veh_velocity_topicname, Odometry,
                        self.extractEgoVehVelocity, queue_size=1)
        rospy.Subscriber(self.getAccel_topicname, CarlaEgoVehicleStatus,
                        self.getAccel, queue_size=1)

    def getAEBParams(self, AEB_Flag):
        if AEB_Flag == 1:
            relative_dist = min(self.drivable_distance, self.MIO_position)
            relative_vel = self.MIO_velocity
            ACC_set_speed = 20                                                  # constant for now should change later 
            if relative_vel != 0:
                ttc = relative_dist/relative_vel
            else:
                ttc = float('inf')
            ego_acc = self.acceleration
            driver_brake = 0
        elif AEB_Flag == 0:
            relative_dist = 1000
            relative_vel = self.ego_velocity_x
            print(relative_vel, "Relative vel")
            ACC_set_speed = 20                                                # constant for now should change later 
            ttc = float('inf')
            ego_acc = self.acceleration
            driver_brake = 0
        dt = rospy.get_time() - self.prv_dt # add time stamps
        self.prv_dt = rospy.get_time()
        return relative_dist, relative_vel, ACC_set_speed, ttc, ego_acc, driver_brake, dt

# MIO: MOST IMPORTANT OBJECT

    def loadParameters(self):
        self.perception_data_topic_name = rospy.get_param("perception_adas_conn/perception_data", "/perception/FCW") 
        self.road_seg_topicname = rospy.get_param("perception_adas_conn/deadahead", "/road/deadahead") 
        self.ego_veh_velocity_topicname = rospy.get_param("perception_adas_conn/ego_velocity","/carla/ego_vehicle/odometry")
        self.getAccel_topicname = rospy.get_param("perception_adas_conn/ego_accel", "/carla/ego_vehicle/vehicle_status")
        self.pub_topic_name = rospy.get_param("perception_adas_conn/adas_features_control", "/vehicle/controls") 

    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.ControlsPublisher = rospy.Publisher(
            self.pub_topic_name, Twist, queue_size=1)

    def extractDataMIO(self, obj_data):
        if obj_data.MIO.class_.data == "None":
            if obj_data.drivable_area.linear.x == 1:
                control_message = Twist()
                control_message.linear.x = 0
                control_message.linear.y = 1
                print("Road nearing end, Brakes engaged!")

                self.callPublisher(control_message)
            elif obj_data.drivable_area.linear.x == 0:
                control_message = Twist()
                # control_message.linear.x = 1
                # control_message.linear.y = 0
                # self.callPublisher(control_message)
                print("ACC FLAG 1")
                relative_dist, relative_vel, ACC_set_speed, ttc, ego_acc, driver_brake, dt = self.getAEBParams(AEB_Flag=0)
                print(relative_dist, relative_vel)
                brake, throttle = self.aeb.get_controls(relative_dist, relative_vel, ACC_set_speed, ttc, ego_acc, driver_brake, dt, acc_flag=1)
                # brake, throttle = self.aeb.get_controls(1000, self.ego_velocity_x, 80, 1200, self.acceleration, 0, dt)

                brake = 1-math.exp(-1*brake)
                brake = max(0, min(1, brake))
                throttle = 1-math.exp(-1*throttle)
                throttle = max(0, min(1, throttle))
                control_message.linear.x = throttle
                control_message.linear.y = brake
                self.callPublisher(control_message)

        elif obj_data.MIO.class_.data == "Some":
            self.MIO_velocity = obj_data.MIO.object_state_dt.y
            self.MIO_position = obj_data.MIO.position.y
            self.go_flag = obj_data.drivable_area.linear.x ## FLAG in linear.x
            self.drivable_distance = obj_data.drivable_area.linear.y
            self.Algorithm()

    def extractEgoVehVelocity(self, odometry):

        self.ego_velocity_x = odometry.twist.twist.linear.x
        # print("Ego Velocity", self.ego_velocity_x)
        self.ego_velocity_y = odometry.twist.twist.linear.y
        self.ego_velocity_angular = odometry.pose.pose.orientation.z
        # print("EGO VEL RECEIVED")

    def getAccel(self, acceleration):
        self.acceleration = acceleration.acceleration.linear.x
        # print("EGO ACC RECEIVED")

    def Algorithm(self):
        control_message = Twist()
        relative_dist, relative_vel, ACC_set_speed, ttc, ego_acc, driver_brake, dt = self.getAEBParams(AEB_Flag=1)
        
        brake, throttle = self.aeb.get_controls(relative_dist, relative_vel, ACC_set_speed, ttc, ego_acc, driver_brake, dt, acc_flag=1)
        # brake = self.stable_sigmoid(brake)
        # throttle = self.stable_sigmoid(throttle)
        # brake, throttle = brake/2, throttle/2
        brake = 1-math.exp(-1*brake)
        brake = max(0, min(1, brake))
        throttle = 1-math.exp(-1*throttle)
        throttle = max(0, min(1, throttle))
        # print(brake, throttle)
        if brake != 0:
            print("On a collision path, Brakes engaged!")
        # Fill twist message here ###
        control_message.linear.x = throttle
        control_message.linear.y = brake
        self.callPublisher(control_message)

    def callPublisher(self, controls):
        self.ControlsPublisher.publish(controls)
        print("Published AEB Message")

# object - velocity, pos
# road-seg - geometry/twist
# vehicle_velot - odometry
# vehicle_accel - CarlaVehicleStatus/Accel
