#!/usr/bin/env python3

import queue
import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# from av_messages.msg import depthandimage

from YOLOP.tools.yolo_p import YOLOP_Class

import torch

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
        
    # def subscribeToTopics(self):
    #     rospy.loginfo("Subscribed to topics")
    #     rospy.Subscriber(self.image_topicname, depthandimage,
    #                      self.storeImage, queue_size=1)

    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")
        rospy.Subscriber(self.image_topicname, Image,
                         self.storeImage, queue_size=1)
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

    def storeImage(self, img):
        if self.RGB_IMAGE_RECEIVED == 0:
            try:
                frame = self.bridge.imgmsg_to_cv2(img, 'bgr8')
                # rospy.loginfo("RGB Image Stored")
            except CvBridgeError as e:
                rospy.loginfo(str(e))
            self.rgb_image = frame
            self.RGB_IMAGE_RECEIVED = 1
            self.sync_frames()

    def storeDepthImage(self, img):
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

    def sync_frames(self):
        if self.RGB_IMAGE_RECEIVED == 1 and self.DEPTH_IMAGE_RECEIVED == 1:
            self.callSegmentationModel(self.rgb_image, self.depth_image)
            self.DEPTH_IMAGE_RECEIVED = 0
            self.RGB_IMAGE_RECEIVED = 0

    def getCartesianPositions(self, depth_img, center_point, CX, FX):
        # perpendicular and lateral distance of car from point
        d = depth_img[center_point[1]][center_point[0]]
        x = (center_point[0] - CX) * d / FX
        return x, d

    def calculateParamsForDistance(self, img_cv):
        FOV = 90   # FOV of camera used
        H, W = img_cv.shape[:2]
        # print(H, W, "H, W")
        CX = W / 2  # center of x-axis
        FX = CX / (np.tan(FOV / 2))  # focal length of x-axis
        orig_dim = (H, W)

        return orig_dim, CX, FX

    
    def callSegmentationModel(self, image, depth_image):
        '''
        Call the segmentation model related functions here (Reuben, Mayur)
        and the final publish function (To be done by sahil)
        '''
        with torch.no_grad():
            points = []
            img_det, da_seg_mask = self.yolo_p.detect(image)
            # print(da_seg_mask.shape, "Da seg mask")
            depth_resized = cv2.resize(depth_image, (480, 640))
            # depth_resized = cv2.resize(depth_image, (640, 480))

            # print(depth_resized.shape, "Depth")

            orig_dim, CX, FX = self.calculateParamsForDistance(depth_resized)
            # print(CX)
            fields = [PointField('x', 0, 7, 1),
                    PointField('y', 4, 7, 1),
                    PointField('z', 8, 7, 1),
                    PointField('intensity', 12, 7, 1)]
            # print(depth_resized.shape)
            # print(da_seg_mask)
            # print(da_seg_mask.shape, "DA SEG MASK")
            # print(depth_resized.shape, "Depth")
            for x in range(len(da_seg_mask)):
                # x = x + 20
                for y in range(len(da_seg_mask[x])):
                    
                    # y = y + 10
                    if da_seg_mask[x][y] == 1:
                        # print(y, x, "y, x")
                        # depth = depth_resized[x][y]
                        
                        depth = depth_resized[y][x]

                        lateral = (x - CX) * depth / FX
                        points.append((depth, lateral, 0, 1))
            header = Header()
            header.stamp = rospy.Time.now()    
            header.frame_id = 'ego_vehicle/rgb_front'
            pc2 = point_cloud2.create_cloud(header, fields, points)
            pc2.header.stamp = rospy.Time.now()
            self.PC2Publisher.publish(pc2)
            self.callPublisher(img_det)

    def callPublisher(self, image):
        '''
        the final publisher function
        '''
        segmented_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.DetectionsPublisher.publish(segmented_image)