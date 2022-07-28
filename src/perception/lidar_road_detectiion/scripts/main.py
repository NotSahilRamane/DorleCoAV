#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs import msg
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
import ros_numpy
import open3d
import time
from PointCloudConversion import convertCloudFromRosToOpen3d, convertCloudFromOpen3dToRos

pointT_publisher = rospy.Publisher('/ground_cloud', PointCloud2, queue_size=1)


def plot_callback(data):
    global pointT_publisher

    pcd = convertCloudFromRosToOpen3d(data)
    # open3d.visualization.draw_geometries([pcd])
    # print(f"Points before downsampling: {len(pcd.points)} ")
    # downpcd=pcd.voxel_down_sample(voxel_size=0.5)

    # print(f"Points after downsampling: {len(downpcd.points)}")# DOWNSAMPLING
    # open3d.visualization.draw_geometries([downpcd])
    _, inliers = pcd.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=150)
    inlier_cloud=pcd.select_by_index(inliers)
    # outlier_cloud=pcd.select_by_index(inliers,invert=True)
    # inlier_cloud.paint_uniform_color([1,0,0])
    # outlier_cloud.paint_uniform_color([0,0,1])
    # open3d.visualization.draw_geometries([inlier_cloud])
    pointcloud_message = convertCloudFromOpen3dToRos(inlier_cloud, frame_id="ego_vehicle/lidar")
    pointT_publisher.publish(pointcloud_message)



def main():

    rospy.init_node('lidar_road_detecti ion')

    while not rospy.is_shutdown():
        rospy.Subscriber('/carla/ego_vehicle/lidar', PointCloud2,
                         callback=plot_callback) 
        rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
