#!/usr/bin/env python3
import rospy
from road_segmentation import Detector


def main():
    rospy.init_node('road_segmentation')
    detector = Detector()
    detector.subscribeToTopics()
    detector.publishToTopics()
    rospy.spin()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
