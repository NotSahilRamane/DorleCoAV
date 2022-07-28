#!/usr/bin/env python33
import rospy
from lane_detector import Detector

def main():
    rospy.init_node('lane_detector')
    detector = Detector()
    detector.subscribeToTopics()
    detector.publishToTopics()
    rospy.spin()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
