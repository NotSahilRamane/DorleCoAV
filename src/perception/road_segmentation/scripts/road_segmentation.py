#!/usr/bin/env python33

import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from av_messages.msg import depthandimage

import numpy as np
import math

class Detector:
    def __init__(self):
        self.loadParameters()
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        
    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")
        rospy.Subscriber(self.image_topicname, depthandimage,
                         self.storeImage, queue_size=1)

    def loadParameters(self):
        '''
        do something
        '''
        self.image_topicname = rospy.get_param(
            "traffic_lights_detector/image_topic_name", "/camera/imagedata")
        self.pub_topic_name = rospy.get_param(
            "lane_detector/road_segmentation_topic_name", "/camera/roadsegmentation")
    
    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.DetectionsPublisher = rospy.Publisher(
            self.pub_topic_name, "Will have to think about this", queue_size=1)

    def storeImage(self, img):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(img.rgb_image, 'bgr8')
            self.depth_image = self.bridge.imgmsg_to_cv2(img.depth_image, "32FC1") ## Confirm these once
            self.callDetector()
        except CvBridgeError as e:
            rospy.loginfo(str(e))
    
    def callSegmentationModel(self):
        '''
        Call the segmentation model related functions here (Reuben, Mayur)
        and the final publish function (To be done by sahil)
        '''

    def callPublisher(self):
        '''
        the final publisher function
        '''