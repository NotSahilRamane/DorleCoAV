#!/usr/bin/env python33

from email import generator
import rospy
from stereo_depth import DepthGenerator


def main():
    rospy.init_node('depth_image_generator')
    depth_generator = DepthGenerator()
    depth_generator.publishToTopics()
    while not rospy.is_shutdown():
        depth_generator.getImagesFromCam()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
