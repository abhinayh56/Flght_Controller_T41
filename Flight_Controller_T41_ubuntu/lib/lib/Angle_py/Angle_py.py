#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import imu

global imu
des_vel = IMU()

pub = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=100)

def imu_cb(msg):
    global X, Y

def main():
    rospy.init_node('p3dx_pub_N_sub', anonymous=True)
    rospy.Subscriber('/mavros/imu_data', IMU, imu_cb)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

