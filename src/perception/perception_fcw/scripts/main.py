#!/usr/bin/env python3
import rospy
from perception_fcw import Detector


def main():
    rospy.init_node('perception_fcw')
    detector = Detector()
    detector.subscribeToTopics()
    detector.publishToTopics()
    rospy.spin()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
