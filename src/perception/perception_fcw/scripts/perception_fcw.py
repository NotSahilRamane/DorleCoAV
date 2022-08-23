#!/usr/bin/env python3

import queue
import cv2
from matplotlib.pyplot import flag
import rospy
import torch

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# from av_messages.msg import depthandimage
from geometry_msgs.msg import Twist
from YOLOP.tools.yolo_p import YOLOP_Class
from av_messages.msg import object_
from geometry_msgs.msg import Point
# from YOLOP.tools.yolo_p import YOLOP_Class
from Tracking_DeepSORT.tracking_and_depth import DeepSORT
from av_messages.msg import fcw_perception

import numpy as np
import time

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
        self.deepsort = DeepSORT(class_names_file='/home/dorleco/DorleCoAV/src/perception/perception_fcw/scripts/Tracking_DeepSORT/data/labels/coco.names', 
                                yolo_model='/home/dorleco/DorleCoAV/src/perception/perception_fcw/scripts/Tracking_DeepSORT/deep_sort/onnx_models/yolov5s.onnx',
                                model_filename='/home/dorleco/DorleCoAV/src/perception/perception_fcw/scripts/Tracking_DeepSORT/model_data/mars-small128.pb', visualize=True)
        self.last_obj_pos_depth = 0
        self.last_obj_pos_lateral = 0
        self.last_time = 0
        self.id_to_track = []
        self.loop_number = 0
        

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
            "perception_fcw/image_topic_name", "/carla/ego_vehicle/rgb_front/image")
        self.depth_image_topicname = rospy.get_param(
            "perception_fcw/depth_image_topic_name", "/carla/ego_vehicle/depth_front/image")
        self.ROAD_seg_pub_topic_name = rospy.get_param(
            "perception_fcw/road_segmentation_topic_name", "/camera/roadsegmentation")
        self.pub_topic_name = rospy.get_param(
            "perception_fcw/pub_image_topic_name", "/road/detected_image")

    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.roadSegPublisher = rospy.Publisher(
            self.ROAD_seg_pub_topic_name, Image, queue_size=1)
        self.toFCWPublisher = rospy.Publisher("/perception/FCW", Point, queue_size=1)
        self.DetectionsPublisher = rospy.Publisher(
            self.pub_topic_name, Image, queue_size=1)

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
            fcw_message = Point()
            rgb_image1, depth_image1 = np.copy(self.rgb_image), np.copy(self.depth_image)
            drivable_area = self.callSegmentationModel(self.rgb_image, self.depth_image)
            rel_distance, rel_velocity = self.callObjectDetector(rgb_image1, depth_image1)
            
            if rel_distance == -100:
                fcw_message.x = drivable_area
                fcw_message.y = 0
                fcw_message.z = 0
            else:
                if drivable_area < rel_distance:
                    fcw_message.x = drivable_area
                    fcw_message.y = 0
                    fcw_message.z = 0
                else:
                    fcw_message.x = rel_distance
                    fcw_message.y = rel_velocity
                    fcw_message.z = 0

            self.toFCWPublisher.publish(fcw_message)
            # print("FCW Published")
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
                                # if lateral in [-1.0, -0.5, 0.0, 0.5, 1.0]:
                                if depth > max_depth:
                                    max_depth = depth
                                    # print("Depth", depth)


            print("Published")
            self.callPublisher(image_detections)
            return max_depth


    def callPublisher(self, image):
        '''
        the final publisher function
        '''
        segmented_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.roadSegPublisher.publish(segmented_image)

    def callObjectDetector(self, image, depth_image): 
        '''
        Call the segmentation model related functions here (Reuben, Mayur)
        and the final publish function (To be done by sahil)
        '''
        # print("Tracker called")
        obj_message = object_()
        img, tracked_boxes = self.deepsort.do_object_detection(image)
        # print(tracked_boxes, "Tracked boxes")  
        relative_distance = 0
        relative_velocity = 0
        orig_dim, CX, FX = self.calculateParamsForDistance(depth_image)
        self.callObjectTrackingPub(img)
        # print("Image Published")
        if tracked_boxes != []:
            for x, y, id in tracked_boxes:
                # print("Inside forloop main")
                if self.loop_number == 0: ## PUBLISH OBJECT MESSAGE FOR MIO INITIAL LOOP, NO VELOCITY
                    # print("loop 0")
                    depth = depth_image[y][x] # instead of x, y, give pixel coordinates of Bounding boxes
                    lateral = (x - CX) * depth / FX
                    # print(lateral, depth, id, "Veh coordinates")
                    if lateral > -1.5 and lateral < 1.5:
                        relative_distance = depth
                        self.id_to_track.append(id)
                        # print(self.id_to_track, "ID TO TRACK")

                        # print(lateral, depth)
                        self.loop_number = 1
                        # self.MIOPublisher.publish(obj_message)
                        print("published loop 0")
                        self.last_time = time.time()

                elif self.loop_number == 1: ## PUBLISH OBJECT MESSAGE FOR MIO Later LOOPs, With VELOCITY
                    # print("loop 1")
                    # print(self.id_to_track)

                    if id == self.id_to_track[-1]:
                        depth = depth_image[y][x] # instead of x, y, give pixel coordinates of Bounding boxes
                        lateral = (x - CX) * depth / FX
                        if lateral > -1.5 and lateral < 1.5:
                            relative_distance = depth
                            this_time = time.time()
                            relative_velocity = (depth - self.last_obj_pos_depth) / (this_time - self.last_time) ## Rvx_act
                            self.last_obj_pos_depth = depth

                            self.last_time = this_time
                    else:                                             ## NEW MIO ID Detected
                        depth = depth_image[y][x] # instead of x, y, give pixel coordinates of Bounding boxes 
                        lateral = (x - CX) * depth / FX
                        if lateral > -1.5 and lateral < 1.5:
                            relative_distance = depth
                            self.last_obj_pos_depth = depth
                            self.last_obj_pos_lateral = lateral
                            self.id_to_track.append(id)

                            self.loop_number = 1
                            self.last_time = time.time()

            return relative_distance, relative_velocity
        else:
            return -100, -100

    def empty_obj_msg(self):
        obj_message = object_()
        obj_message.class_.data = "None"
        obj_message.position.x = 0
        obj_message.position.y = 0
        obj_message.id.data = 0
        obj_message.object_state_dt.x = 0
        obj_message.object_state_dt.y = 0
        self.last_obj_pos_depth = 0
        self.last_obj_pos_lateral = 0
        obj_message.object_state_dt.theta = 0
        obj_message.position.z = 0

        return obj_message

    def callObjectTrackingPub(self, image):
        '''
        the final publisher function
        '''
        segmented_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        # if segmented_image:
            # print("Segmented")
        self.DetectionsPublisher.publish(segmented_image)