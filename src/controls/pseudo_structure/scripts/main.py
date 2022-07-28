#!/usr/bin/env python33

import rospy
import sample_struct


def main():
    rospy.init_node('demo')
    handle = sample_struct.Sample_publisher()
    handle.publishToTopics()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        messageToSend = handle.message_tobe_sent + str(rospy.get_time())
        handle.messagePublisher.publish(messageToSend)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass