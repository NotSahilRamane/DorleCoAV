#!/usr/bin/env python3
import rospy 
from adas_features import ADAS_Features

def main():
    rospy.init_node('adas_features')
    features = ADAS_Features()
    features.subscribeToTopics()
    features.publishToTopics()
    rospy.spin()

if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass