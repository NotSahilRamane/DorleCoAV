#!/usr/bin/env python3

import this
import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# from av_messages.msg import depthandimage
from av_messages.msg import object
# from YOLOP.tools.yolo_p import YOLOP_Class
from Tracking_DeepSORT.tracking_and_depth import DeepSORT

# import torch
import time
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
        # self.yolo_p = YOLOP_Class()
        self.deepsort = DeepSORT(class_names_file='/home/sahil/DorleCoAV/src/perception/object_tracking/scripts/Tracking_DeepSORT/data/labels/coco.names', 
                                yolo_model='/home/sahil/DorleCoAV/src/perception/object_tracking/scripts/Tracking_DeepSORT/deep_sort/onnx_models/yolov5s.onnx',
                                model_filename='/home/sahil/DorleCoAV/src/perception/object_tracking/scripts/Tracking_DeepSORT/model_data/mars-small128.pb', visualize=True)
        self.RGB_IMAGE_RECEIVED = 0
        self.DEPTH_IMAGE_RECEIVED = 0
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
            "object_tracking/image_topic_name", "/carla/ego_vehicle/rgb_front/image")
        self.depth_image_topicname = rospy.get_param(
            "object_tracking/depth_image_topic_name", "/carla/ego_vehicle/depth_front/image")
        self.pub_topic_name = rospy.get_param(
            "object_tracking/pub_image_topic_name", "/road/detected_image")
    
    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.DetectionsPublisher = rospy.Publisher(
            self.pub_topic_name, Image, queue_size=1)
        self.MIOPublisher = rospy.Publisher("/road/mio", object, queue_size=1)

    def storeImage(self, img): # Copy for Obj Detection
        if self.RGB_IMAGE_RECEIVED == 0:
            try:
                frame = self.bridge.imgmsg_to_cv2(img, 'bgr8')
                rospy.loginfo("RGB Image Stored")
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
                rospy.loginfo("Depth Image Stored")
            except CvBridgeError as e:
                rospy.loginfo(str(e))
            self.depth_image = frame
            self.DEPTH_IMAGE_RECEIVED = 1
            self.sync_frames()

    def sync_frames(self): # Copy for Obj Detection
        if self.RGB_IMAGE_RECEIVED == 1 and self.DEPTH_IMAGE_RECEIVED == 1:
            self.callObjectDetector(self.rgb_image, self.depth_image)
            self.DEPTH_IMAGE_RECEIVED = 0
            self.RGB_IMAGE_RECEIVED = 0

    def calculateParamsForDistance(self, img_cv): # Copy for Obj Detection
        FOV = 90   # FOV of camera used
        H, W = img_cv.shape[:2]
        # print(H, W, "H, W")
        CX = W / 2  # center of x-axis
        FX = CX / (2*np.tan(FOV*3.14 /360))  # focal length of x-axis
        orig_dim = (H, W)

        return orig_dim, CX, FX
    
    def callObjectDetector(self, image, depth_image): # Copy for Obj Detection remove for loops
        '''
        Call the segmentation model related functions here (Reuben, Mayur)
        and the final publish function (To be done by sahil)
        '''
        print("Tracker called")
        obj_message = object()
        img, tracked_boxes = self.deepsort.do_object_detection(image)

        # depth_resized = cv2.resize(depth_image, (640, 480), interpolation=cv2.INTER_AREA)   

        orig_dim, CX, FX = self.calculateParamsForDistance(depth_image)

        for x, y, id in tracked_boxes:
            if self.loop_number == 0:
                depth = depth_image[x][y] # instead of x, y, give pixel coordinates of Bounding boxes
                lateral = (y - CX) * depth / FX
                if lateral > -0.5 and lateral < 0.5:
                    self.last_obj_pos_depth = depth
                    self.last_obj_pos_lateral = lateral
                    self.id_to_track.append(id)
                    obj_message.position.x = lateral
                    obj_message.position.y = depth
                    obj_message.id = id
                    obj_message.object_state_dt.x = 0
                    obj_message.object_state_dt.y= 0
                    self.MIOPublisher.publish(obj_message)
                    self.loop_number = 1
                    self.last_time = time.time()
            elif self.loop_number == 1:
                if id == self.id_to_track[-1]:
                    depth = depth_image[x][y] # instead of x, y, give pixel coordinates of Bounding boxes
                    lateral = (y - CX) * depth / FX
                    if lateral > -0.5 and lateral < 0.5:
                        self.last_obj_pos_depth = depth
                        self.last_obj_pos_lateral = lateral
                        obj_message.position.x = lateral
                        obj_message.position.y = depth
                        obj_message.id = id
                        this_time = time.time()
                        obj_message.object_state_dt.x = (lateral - self.last_obj_pos_lateral) / (this_time - self.last_time)
                        obj_message.object_state_dt.x = (lateral - self.last_obj_pos_lateral) / (this_time - self.last_time)
                        self.MIOPublisher.publish(obj_message)
                        self.last_obj_pos_depth = depth
                        self.last_obj_pos_lateral = lateral
                        self.last_time = this_time
                else:
                    depth = depth_image[x][y] # instead of x, y, give pixel coordinates of Bounding boxes
                    lateral = (y - CX) * depth / FX
                    if lateral > -1.5 and lateral < 1.5:
                        self.last_obj_pos_depth = depth
                        self.last_obj_pos_lateral = lateral
                        self.id_to_track.append(id)
                        obj_message.position.x = lateral
                        obj_message.position.y = depth
                        obj_message.id = id
                        obj_message.object_state_dt.x = 0
                        obj_message.object_state_dt.y= 0
                        self.MIOPublisher.publish(obj_message)
                        self.loop_number = 1
                        self.last_time = time.time()

        self.callPublisher(img)


    def callPublisher(self, image):
        '''
        the final publisher function
        '''
        segmented_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        if segmented_image:
            print("Segmented")
        self.DetectionsPublisher.publish(segmented_image)