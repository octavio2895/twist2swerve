#!/usr/bin/env python

import math as M
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time
import rospy
from geometry_msgs.msg import Twist
#Cheif Delphi Help-
#https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
#https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
cb_tup = (0, 0, 0)

def twist_callback(data):
    global cb_tup
    cb_tup = (data.linear.x, data.linear.y, data.angular.z)

class SwerveDrive:
    w_b = 0 #"width" between wheel centerpoints
    l_b = 0 #"length" between axels
    inpuvectors = np.zeros((4,2))
    def __init__(self, w_b, l_b,):
        self.w_b = w_b
        self.l_b = l_b

    def drive(self, x_v, y_v, yaw):
        #translation velocities (x_v, y_v) are between 0 and 1
        a = x_v - (self.w_b/2)*yaw #defined as the sum of translation velocity (x_v) and the rotation velocity (r times angular frequency (yaw))
        b = x_v + (self.w_b/2)*yaw
        c = y_v - (self.l_b/2)*yaw
        d = y_v + (self.l_b/2)*yaw
        #starting from the wheel in the top right as 1 and going COUNTERclockwise in ascending order
        vector_output = np.array([[truncate(M.sqrt(pow(b,2)+pow(c,2)),3), M.atan2(b,c)*180/(M.pi)],
                                [truncate(M.sqrt(pow(a,2)+pow(c,2)),3), M.atan2(a,c)*180/(M.pi)],
                                [truncate(M.sqrt(pow(a,2)+pow(d,2)),3), M.atan2(a,d)*180/(M.pi)],
                                [truncate(M.sqrt(pow(b,2)+pow(d,2)),3), M.atan2(b,d)*180/(M.pi)]])
        return vector_output

def truncate(number, digits) -> float:
    # Improve accuracy with floating point operations, to avoid truncate(16.4, 2) = 16.39 or truncate(-1.13, 2) = -1.12
    nbDecimals = len(str(number).split('.')[1])
    if nbDecimals <= digits:
        return number
    stepper = 10.0 ** digits
    return M.trunc(stepper * number) / stepper

def swerve():
    global cb_tup
    #print("Test")
    #time.sleep(5)
    rospy.init_node('twist2_new', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber("motor12345678", Twist, twist_callback)

    JohnnyBot = SwerveDrive(1,1)


    while not rospy.is_shutdown():
        vector_output = JohnnyBot.drive(cb_tup[0],cb_tup[1],cb_tup[2])
        print(vector_output)

        (rospy.Publisher("vel_l_top_r", Float32, queue_size = 10)).publish(vector_output[0][0])
        (rospy.Publisher("vel_l_top_l", Float32, queue_size = 10)).publish(vector_output[1][0])
        (rospy.Publisher("vel_l_bottom_l", Float32, queue_size = 10)).publish(vector_output[2][0])
        (rospy.Publisher("vel_l_bottom_r", Float32, queue_size = 10)).publish(vector_output[3][0])
        (rospy.Publisher("vel_r_top_r", Float32, queue_size = 10)).publish(vector_output[0][1])
        (rospy.Publisher("vel_r_top_l", Float32, queue_size = 10)).publish(vector_output[1][1])
        (rospy.Publisher("vel_r_bottom_l", Float32, queue_size = 10)).publish(vector_output[2][1])
        (rospy.Publisher("vel_r_bottom_r", Float32, queue_size = 10)).publish(vector_output[3][1])
        rate.sleep()

if __name__ == "__main__":
    try:
        swerve()
    except  rospy.ROSInterruptException: pass
