#!/usr/bin/env python3

import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# from av_messages.msg import depthandimage

from YOLOP.tools.yolo_p import YOLOP_Class


import numpy as np
import math

class Detector:
    def __init__(self):
        self.loadParameters()
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.yolo_p = YOLOP_Class()
        
    # def subscribeToTopics(self):
    #     rospy.loginfo("Subscribed to topics")
    #     rospy.Subscriber(self.image_topicname, depthandimage,
    #                      self.storeImage, queue_size=1)

    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")
        rospy.Subscriber(self.image_topicname, Image,
                         self.storeImage, queue_size=1)

    def loadParameters(self):
        '''
        do something
        '''
        self.image_topicname = rospy.get_param(
            "camera_object_detector/image_topic_name", "/carla/ego_vehicle/rgb_view/image")
        self.pub_topic_name = rospy.get_param(
            "lane_detector/road_segmentation_topic_name", "/camera/roadsegmentation")
    
    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.DetectionsPublisher = rospy.Publisher(
            self.pub_topic_name, Image, queue_size=1)

    def storeImage(self, img):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(img, 'bgr8')
            # self.depth_image = self.bridge.imgmsg_to_cv2(img.depth_image, "32FC1") ## Confirm these once
            self.callSegmentationModel()
        except CvBridgeError as e:
            rospy.loginfo(str(e))
    
    def callSegmentationModel(self):
        '''
        Call the segmentation model related functions here (Reuben, Mayur)
        and the final publish function (To be done by sahil)
        '''
        img_det, _, _ = self.yolo_p.detect(self.rgb_image)
        self.callPublisher(img_det)

    def callPublisher(self, image):
        '''
        the final publisher function
        '''
        segmented_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.DetectionsPublisher.publish(segmented_image)