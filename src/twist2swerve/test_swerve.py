#!/usr/bin/env python3

import math as M
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time
import rospy
from geometry_msgs.msg import TwistStamped
#Cheif Delphi Help-
#https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
#https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
cb_twist = Twist()

def twist_callback(data):
    global cb_twist
    cb_twist = data
    cb_tup = (data.linear.x, data.linear.y, data.angular.z)

class SwerveDrive:
    w_b = 0 #"width" between wheel centerpoints
    l_b = 0 #"length" between axels
    inpuvectors = np.zeros((4,2))
    def __init__(self, w_b, l_b, diam, poles):
        self.w_b = w_b
        self.l_b = l_b
        self.diam = diam
        self.poles = poles

    def drive(self, twist):
        #translation velocities (x_v, y_v) are between 0 and 1
        x_v = twist.linear.x
        y_v = twist.linear.y
        yaw = twist.angular.z
        ms_to_erpm = self.poles*60*(1/(self.diam*np.pi))
        r = M.sqrt(self.w_b**2 + self.l_b**2)
        a = y_v - (self.l_b/r)*yaw #defined as the sum of translation velocity (x_v) and the rotation velocity (r times angular frequency (yaw))
        b = y_v + (self.l_b/r)*yaw
        c = x_v - (self.w_b/r)*yaw
        d = x_v + (self.w_b/r)*yaw
        vector_output = np.zeros([4,2], dtype=np.float64)
        #starting from the wheel in the top right as 1 and going COUNTERclockwise in ascending order
        # vector_output = np.array([[truncate(M.sqrt(pow(b,2)+pow(c,2)),3), M.atan2(b,c)*180/(M.pi)],
        #                         [truncate(M.sqrt(pow(a,2)+pow(c,2)),3), M.atan2(a,c)*180/(M.pi)],
        #                         [truncate(M.sqrt(pow(a,2)+pow(d,2)),3), M.atan2(a,d)*180/(M.pi)],
        #                         [truncate(M.sqrt(pow(b,2)+pow(d,2)),3), M.atan2(b,d)*180/(M.pi)]])
        vector_output[0][0] = truncate(M.sqrt(pow(b,2)+pow(c,2)),3)*ms_to_erpm
        vector_output[0][1] = M.atan2(b,c)*180/(M.pi)+180
        vector_output[0][0], vector_output[0][1] = self.check_bounds(vector_output[0][0], vector_output[0][1])
        vector_output[1][0] = truncate(M.sqrt(pow(a,2)+pow(c,2)),3)*ms_to_erpm
        vector_output[1][1] = M.atan2(a,c)*180/(M.pi)+180
        vector_output[1][0], vector_output[1][1] = self.check_bounds(vector_output[1][0], vector_output[1][1])
        vector_output[2][0] = truncate(M.sqrt(pow(a,2)+pow(d,2)),3)*ms_to_erpm
        vector_output[2][1] = M.atan2(a,d)*180/(M.pi)+180
        vector_output[2][0], vector_output[2][1] = self.check_bounds(vector_output[2][0], vector_output[2][1])
        vector_output[3][0] = truncate(M.sqrt(pow(b,2)+pow(d,2)),3)*ms_to_erpm
        vector_output[3][1] = M.atan2(b,d)*180/(M.pi)+180
        vector_output[3][0], vector_output[3][1] = self.check_bounds(vector_output[3][0], vector_output[3][1])
        return vector_output

    def check_bounds(self, erpm, yaw):
        MAX_STEER_ANG = 270
        MIN_STEER_ANG = 90
        if yaw <= MAX_STEER_ANG and yaw >= MIN_STEER_ANG:
            return erpm, yaw
        else:
            if yaw > MAX_STEER_ANG:
                yaw = yaw - 180
                erpm = - erpm
            elif yaw < MIN_STEER_ANG:
                yaw = yaw + 180
                erpm = -erpm
        return erpm, yaw
        

def truncate(number, digits) -> float:
    # Improve accuracy with floating point operations, to avoid truncate(16.4, 2) = 16.39 or truncate(-1.13, 2) = -1.12
    nbDecimals = len(str(number).split('.')[1])
    if nbDecimals <= digits:
        return number
    stepper = 10.0 ** digits
    return M.trunc(stepper * number) / stepper

def swerve():
    global cb_twist
    #print("Test")
    #time.sleep(5)
    rospy.init_node('twist2swerve', anonymous=False)
    rate = rospy.Rate(10)
    rospy.Subscriber("/cmd_vel/remote_joystick", Twist, twist_callback)

    JohnnyBot = SwerveDrive(0.45,0.45, 0.2, 14)

    while not rospy.is_shutdown():
        vector_output = JohnnyBot.drive(cb_twist)
        # print(vector_output)

        (rospy.Publisher("motor_2/speed", Float64, queue_size = 10)).publish(vector_output[0][0]) #vel_l_top_r
        (rospy.Publisher("motor_1/speed", Float64, queue_size = 10)).publish(vector_output[3][0]) #vel_l_top_l
        (rospy.Publisher("motor_3/speed", Float64, queue_size = 10)).publish(vector_output[2][0]) #vel_l_bottom_l
        (rospy.Publisher("motor_4/speed", Float64, queue_size = 10)).publish(vector_output[1][0]) #vel_l_bottom_e
        (rospy.Publisher("motor_b/position", Float64, queue_size = 10)).publish(vector_output[0][1]) #vel_r_top_r
        (rospy.Publisher("motor_a/position", Float64, queue_size = 10)).publish(vector_output[3][1]) #vel_r_top_l
        (rospy.Publisher("motor_c/position", Float64, queue_size = 10)).publish(vector_output[2][1]) #vel_r_bottom_l
        (rospy.Publisher("motor_d/position", Float64, queue_size = 10)).publish(vector_output[1][1]) ##vel_r_bottom_r
        rate.sleep()

if __name__ == "__main__":
    try:
        swerve()
    except  rospy.ROSInterruptException: pass
