#!/usr/bin/env python3

import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError
from av_messages.msg import object
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class ADAS_Features:
    def __init__(self):
        self.loadParameters()
        # self.bridge = CvBridge()


    def subscribeToTopics(self):
        rospy.loginfo('Subscribed to topics')
        rospy.Subscriber(self.MOT_object_topicname, object,
                        self.extractDataMOO, queue_size=1)
        rospy.Subscriber(self.road_seg_topicname, Twist,
                        self.callback1, queue_size=1)
        rospy.Subscriber(self.ego_veh_velocity_topicname, Odometry,
                        self.callback2, queue_size=1)

    def loadParameters(self):
        self.MOT_object_topicname = rospy.get_param("")
        self.road_seg_topicname = rospy.get_param("")
        self.ego_veh_velocity_topicname = rospy.get_param("")
        self.pub_topic_name = rospy.get_param("") 
    
    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.ControlsPublisher = rospy.Publisher(
            self.pub_topic_name, Twist, queue_size=1)
        
    def extractDataMOO(self, obj_data):
        MOT_velocity = obj_data.object_state_dt
        MOT_position = obj_data.positionobject_state_dt

    def extractDataRoadSeg(self, seg_data):
        go_flag = seg_data.linear.x
        distance = seg_data.linear.y

    def extractEgoVehVelocity(self, odometry):
        ego_velocity_x = odometry.twist.linear.x
        ego_velocity_y = odometry.twist.linear.y
        ego_velocity_angular = odometry.angular.z

    def Algorithm(self):
        '''
        Add algorithm related fns here
        '''
        controls = 1.0 # just for the time being
        self.callPublisher(controls)

    def callPublisher(self, controls):
        self.ControlsPublisher.publish(controls)

# object - velocity, pos
# road-seg - geometry/twist
# vehicle_velot - odometry