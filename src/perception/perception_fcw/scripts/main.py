#!/usr/bin/env python3
import rospy
from perception_fcw import Detector


def main():
    rospy.init_node('perception_fcw')
    rate = rospy.Rate(3) #hz
    detector = Detector(ros_rate=rate)
    detector.subscribeToTopics()
    detector.publishToTopics()
    # rate.sleep()
    rospy.spin()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
