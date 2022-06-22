#!/usr/bin/env python3
import rospy
from planner import LocalPlanner


def main():
    rospy.init_node('local_planner')
    path_planner = LocalPlanner()
    path_planner.subscribeToTopics()
    path_planner.publishToTopics()
    rospy.spin()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass