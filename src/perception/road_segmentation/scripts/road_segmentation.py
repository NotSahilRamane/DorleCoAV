#!/usr/bin/env python3

import queue
import cv2
import rospy
import torch

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# from av_messages.msg import depthandimage
from geometry_msgs.msg import Twist
from YOLOP.tools.yolo_p import YOLOP_Class


import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class Detector:
    def __init__(self):
        self.loadParameters()
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.yolo_p = YOLOP_Class()
        self.RGB_IMAGE_RECEIVED = 0
        self.DEPTH_IMAGE_RECEIVED = 0

    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")
        rospy.Subscriber(self.image_topicname, Image,
                         self.storeImage, buff_size = 2**24, queue_size=1)
        rospy.Subscriber(self.depth_image_topicname, Image,
                         self.storeDepthImage, queue_size=1)

    def loadParameters(self):
        '''
        do something
        '''
        self.image_topicname = rospy.get_param(
            "road_segmentation/image_topic_name", "/carla/ego_vehicle/rgb_front/image")
        self.depth_image_topicname = rospy.get_param(
            "road_segmentation/depth_image_topic_name", "/carla/ego_vehicle/depth_front/image")
        self.pub_topic_name = rospy.get_param(
            "road_segmentation/road_segmentation_topic_name", "/camera/roadsegmentation")

    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.DetectionsPublisher = rospy.Publisher(
            self.pub_topic_name, Image, queue_size=1)
        self.PC2Publisher = rospy.Publisher("/road/pointcloud", PointCloud2, queue_size=1)
        self.DAOPublisher = rospy.Publisher("/road/deadahead", Twist, queue_size=1)    


    def storeImage(self, img): # Copy for Obj Detection
        if self.RGB_IMAGE_RECEIVED == 0:
            try:
                frame = self.bridge.imgmsg_to_cv2(img, 'bgr8')
                # rospy.loginfo("RGB Image Stored")
            except CvBridgeError as e:
                rospy.loginfo(str(e))
            self.rgb_image = frame
            self.RGB_IMAGE_RECEIVED = 1
            self.sync_frames()

    def storeDepthImage(self, img): # Copy for Obj Detection
        frame=None
        if self.DEPTH_IMAGE_RECEIVED == 0:
            try:
                frame = self.bridge.imgmsg_to_cv2(img, "32FC1")
                # rospy.loginfo("Depth Image Stored")
            except CvBridgeError as e:
                rospy.loginfo(str(e))
            self.depth_image = frame
            self.DEPTH_IMAGE_RECEIVED = 1
            self.sync_frames()

    def sync_frames(self): # Copy for Obj Detection
        if self.RGB_IMAGE_RECEIVED == 1 and self.DEPTH_IMAGE_RECEIVED == 1:
            self.callSegmentationModel(self.rgb_image, self.depth_image)
            self.DEPTH_IMAGE_RECEIVED = 0
            self.RGB_IMAGE_RECEIVED = 0

    def getCartesianPositions(self, depth_img, center_point, CX, FX): # Copy for Obj Detection
        # perpendicular and lateral distance of car from point
        d = depth_img[center_point[1]][center_point[0]]
        x = (center_point[0] - CX) * d / FX
        return x, d

    def calculateParamsForDistance(self, img_cv): # Copy for Obj Detection
        FOV = 90   # FOV of camera used
        H, W = img_cv.shape[:2]
        # print(H, W, "H, W")
        CX = W / 2  # center of x-axis
        FX = CX / (2*np.tan(FOV*3.14 /360))  # focal length of x-axis
        orig_dim = (H, W)

        return orig_dim, CX, FX

    def callSegmentationModel(self, image, depth_image): # Copy for Obj Detection remove for loops
        '''
        Call the segmentation model related functions here (Reuben, Mayur)
        and the final publish function (To be done by sahil)
        '''
        with torch.no_grad():
            points = []
            da_seg_mask, ll_seg_mask, image_detections = self.yolo_p.detect(image)

            depth_resized = cv2.resize(depth_image, (640, 480), interpolation=cv2.INTER_AREA)   

            orig_dim, CX, FX = self.calculateParamsForDistance(depth_resized)

            one_d = 0
            m_one_d = 0
            half_d = 0
            m_half_d = 0
            zero_d = 0
            max_depth = 0
            
            for x in range(0, len(da_seg_mask), 2):
                for y in range(0, len(da_seg_mask[x]), 2):
                    if da_seg_mask[x][y] == 1 or ll_seg_mask[x][y]:
                        depth = depth_resized[x][y] # instead of x, y, give pixel coordinates of Bounding boxes
                        
                        if depth <= 50.00:
                            lateral = (y - CX) * depth / FX
                            if lateral >= -1 and lateral <= 1:
                                # print(lateral)
                                if lateral in [-1.0, -0.5, 0.0, 0.5, 1.0]:
                                    if depth > max_depth:
                                        max_depth = depth
            flag_message = Twist()
            if max_depth <= 30:
                flag_message.linear.x = 1
                flag_message.linear.y = max_depth
            else:
                flag_message.linear.x = 0
                flag_message.linear.y = max_depth
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'ego_vehicle/rgb_front'

            print("Published")
            self.callPublisher(image_detections, flag_message)

            

    def callPublisher(self, image, flag_message):
        '''
        the final publisher function
        '''
        segmented_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.DetectionsPublisher.publish(segmented_image)
        # self.PC2Publisher.publish(pcl2)
        self.DAOPublisher.publish(flag_message)