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
            da_seg_mask = self.yolo_p.detect(image)

            depth_resized = cv2.resize(depth_image, (640, 480), interpolation=cv2.INTER_AREA)   

            orig_dim, CX, FX = self.calculateParamsForDistance(depth_resized)
            
            fields = [PointField('x', 0, 7, 1),
                    PointField('y', 4, 7, 1),
                    PointField('z', 8, 7, 1),
                    PointField('intensity', 12, 7, 1)]
           
            
            # for x in range(len(da_seg_mask)):
            #     for y in range(len(da_seg_mask[x])):
            #         if da_seg_mask[x][y] == 1:
            #             depth = depth_resized[x][y] # instead of x, y, give pixel coordinates of Bounding boxes
            #             if depth <= 30.00:
            #                 lateral = (y - CX) * depth / FX
            #                 if lateral > -0.1 and lateral < 0.1:
            #                     print(lateral, depth)
            #                     points.append((depth, lateral, 0, 1))
            #         if y + 10 > len(da_seg_mask[x]):
            #             y = len(da_seg_mask[x]) - 1
            #         else:
            #             y = y + 10
            #     if x + 10 > len(da_seg_mask):
            #         x = len(da_seg_mask) - 1
                
            header = Header()
            header.stamp = rospy.Time.now()    
            header.frame_id = 'ego_vehicle/rgb_front'
            pc2 = point_cloud2.create_cloud(header, fields, points)
            pc2.header.stamp = rospy.Time.now()
            self.PC2Publisher.publish(pc2)

    def callPublisher(self, image):
        '''
        the final publisher function
        '''
        segmented_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.DetectionsPublisher.publish(segmented_image)