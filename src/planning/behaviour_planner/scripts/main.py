#!/usr/bin/env python3
import rospy
from planner import BehaviourPlanner


def main():
    rospy.init_node('behaviour_planner')
    path_planner = BehaviourPlanner()
    path_planner.subscribeToTopics()
    path_planner.publishToTopics()
    rospy.spin()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass