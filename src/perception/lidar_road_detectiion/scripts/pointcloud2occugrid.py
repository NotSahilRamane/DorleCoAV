#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs import msg
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
import ros_numpy
import open3d
import time
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from PointCloudConversion import convertCloudFromRosToOpen3d

start = time.time()
fig, ax = plt.subplots()
ln = plt.scatter(x=np.array([]), y=np.array(
    []), s=1.0, vmin=0, vmax=1)
x_data, y_data = [], []

def plot_init():
    global ax, ln
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    return ln


def update_plot(frame):
    global start, ln, x_data, y_data
 
    end = time.time()
    if end-start >= 0.2:
        ln.set_offsets(
            np.c_[np.array(x_data), np.array(y_data)])
        start = end
    return ln


def plot_callback(data):
    global x_data, y_data
    x_data, y_data = [], []
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
    xyz = np.asarray([inlier_cloud.points])
    for field in xyz:
        for point in field:
            # print(point)
            x_data.append(point[0])
            y_data.append(point[1])


    



def main():

    rospy.init_node('correction_node')

    while not rospy.is_shutdown():
        rospy.Subscriber('/carla/ego_vehicle/lidar', PointCloud2,
                         callback=plot_callback) 
        ani = FuncAnimation(fig, update_plot, init_func=plot_init)
        plt.show()
        rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
