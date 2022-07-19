#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist

def fakeData():
    pub = rospy.Publisher("/AEB", Twist, queue_size=1)
    rospy.init_node("FakeAEB", anonymous=True)
    while not rospy.is_shutdown():
        mesg = Twist()
        mesg.linear.x = float(input("Enter throttle val"))
        mesg.linear.y = float(input("Enter brake val"))
        pub.publish(mesg)

if __name__ == "__main__":
    try:
        fakeData()
    except rospy.ROSInterruptException:
        pass

