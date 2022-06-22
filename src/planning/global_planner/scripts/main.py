#!/usr/bin/env python3
import rospy
from planner import GlobalPlanner


def main():
    rospy.init_node('global_planner')
    path_planner = GlobalPlanner()
    path_planner.subscribeToTopics()
    path_planner.publishToTopics()
    rospy.spin()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass