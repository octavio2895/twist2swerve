#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
import random

pub = rospy.Publisher('motor12345678', Twist, queue_size=10)

rospy.init_node('testpub', anonymous=True)

rate = rospy.Rate(1)

i = 0
while not rospy.is_shutdown():
    testpub = Twist()
    testpub.linear.x = -1
    testpub.linear.y = -1
    testpub.angular.z = 0
    rospy.loginfo("I publish:")
    rospy.loginfo(testpub)
    pub.publish(testpub)
    rate.sleep()
    i = i+1