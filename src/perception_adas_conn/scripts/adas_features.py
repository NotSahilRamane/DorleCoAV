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
        self.bridge = CvBridge()


    def subscribeToTopics(self):
        rospy.loginfo('Subscribed to topics')
        rospy.Subscriber(self.MOT_object_topicname, object,
                        self.callback, queue_size=1)
        rospy.Subscriber(self.road_seg_topicname, Twist,
                        self.callback1, queue_size=1)
        rospy.Subscriber(self.ego_veh_velocity_topicname, Odometry,
                        self.callback2, queue_size=1)

    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.ControlsPublisher = rospy.Publisher(
            self.pub_topic_name, Twist, queue_size=1)
        

# object - velocity, pos
# road-seg - geometry/twist
# vehicle_velot - odometry