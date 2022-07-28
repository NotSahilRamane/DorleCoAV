#!/usr/bin/env python33


import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from av_messages.msg import depthandimage
import numpy as np
import math

class DepthGenerator(object):
    def __init__(self):
        self.loadParameters()
        self.bridge = CvBridge()
        self.pub_topic_name = ""

    def publishToTopics(self):
        rospy.loginfo("Publish to topics")
        self.imagePublisher = rospy.Publisher(
            self.pub_topic_name, depthandimage, queue_size=1)
        rospy.loginfo("Done Publish")

    def loadParameters(self):
        rospy.loginfo("loading handle parameters")

        self.pub_topic_name = rospy.get_param(
            "stereo_depth/publisher_topic_name", "/camera/imagedata")

    def getImagesFromCam(self):
        '''
        insert the code to access the images from the oakd camera here
        initialise variables needed accordingly
        Get both the mono camera images as well as the rgb camera image in this function
        so as to sync everything
        '''

    def computeDepth(self):
        '''
        insert the code to calculated depth from the obtained set of images
        initialise variables needed accordingly
        '''

    def callPublisher(self):
        '''
        insert the code to save the computed depth and obtained rgb image (Reuben and Mayur)
        and call the publisher function
        to be filled by Sahil after the algorithm is added
        '''
