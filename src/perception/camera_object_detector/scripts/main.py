#!/usr/bin/env python3

import rospy
from camera_object_detector import Detector

def main():
    rospy.init_node('camera_object_detector')
    detector = Detector()
    detector.subscribeToTopics()
    detector.publishToTopics()
    rospy.spin()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
