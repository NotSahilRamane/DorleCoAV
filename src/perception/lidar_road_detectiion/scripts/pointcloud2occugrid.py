#!/usr/bin/env python2

import rospy
import numpy as np
from sensor_msgs import msg
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
import ros_numpy
import open3d

pointT_publisher = rospy.Publisher('/cloud_corrected', PointCloud2, queue_size=1)


def plot_callback(data):

    global pointT_publisher
    pointsT = []
    time1 = rospy.get_time()
    points = ros_numpy.point_cloud2.pointcloud2_to_array(data, squeeze=True)
    for j in range(len(points)):
        for i in range(len(points[j])):
            if points[j][i][1] >= -2 and points[j][i][1] <= 2:
                if points[j][i][0] > 0.3:
                    pointsT.append(points[j][i])
        
    fields = [PointField('x', 0, 7, 1),
                    PointField('y', 4, 7, 1),
                    PointField('z', 8, 7, 1),
                    PointField('intensity', 12, 7, 1)]
    header = Header()
    header.stamp = rospy.Time.now()    
    header.frame_id = 'cloud'
    pc2 = point_cloud2.create_cloud(header, fields, pointsT)
    pc2.header.stamp = rospy.Time.now()
    pointT_publisher.publish(pc2)
    print("Success")

def main():

    rospy.init_node('correction_node')

    while not rospy.is_shutdown():
        rospy.Subscriber('/cloud', PointCloud2,
                         callback=plot_callback) 
        rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
