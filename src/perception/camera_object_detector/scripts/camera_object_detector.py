#!/usr/bin/env python3

import cv2
import rospy
import yolov5

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from av_messages.msg import depthandimage, object, objects


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
            "camera_object_detector/image_topic_name", "/camera/imagedata")
        self.pub_topic_name = rospy.get_param(
            "camera_object_detector/object_detections_topic_name", "/camera/objectdetections")
    

    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.DetectionsPublisher = rospy.Publisher(
            self.pub_topic_name, objects, queue_size=1)

    def storeImage(self, img):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(img.rgb_image, 'bgr8')
            self.depth_image = self.bridge.imgmsg_to_cv2(img.depth_image, "32FC1") ## Confirm these once
            self.callYOLO()
        except CvBridgeError as e:
            rospy.loginfo(str(e))
    
    def callYOLO(self):
        '''
        Call yolo related functions here (Reuben, Mayur)
        and the final publish function (To be done by sahil)
        '''
        yolo = yolov5.YOLO_Fast()
        boxes, scores, classes, num_dets = yolo.object_detection(self.rgb_image)

    def callPublisher(self):
        '''
        the final publisher function
        '''

        