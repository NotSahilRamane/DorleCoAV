#!/usr/bin/env python 3
#import all the libraries
import rospy
from ros_architecture import Ros_architecture

def main():
    rospy.init_node('lateral_lka_lca','anonymous')
    obj=Ros_architecture()
    obj.subscribeToTopics()
    obj.publishToTopics()
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
