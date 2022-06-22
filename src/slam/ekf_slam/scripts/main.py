#!/usr/bin/env python3
import rospy
from ekf_slam import EKFSlam


def main():
    rospy.init_node('ekf_slam')
    slam = EKFSlam()
    slam.subscribeToTopics()
    slam.publishToTopics()
    rospy.spin()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
