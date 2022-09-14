#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time

ranges = []
dist = 0

def Callback(scan_data):
    global ranges
    ranges = list(scan_data.ranges)
    l_ranges = ranges[0:30]
    r_ranges = ranges[330:360]
    global dist
    dist = min(l_ranges + r_ranges)
    l_min = min(l_ranges)
    r_min = min(r_ranges)
    print(l_min)
    print(r_min)


    linear_vel(dist)
    angular_vel(l_min, r_min)
    print(vel)


def linear_vel(distance):
    vel.linear.x = distance * 0.3
    velocity_publisher.publish(vel)
    rate.sleep()



def angular_vel(l_min, r_min):
    global dist
    if (dist < 0.2):
        if (l_min > r_min):
            vel.angular.z = 0.6/l_min
            velocity_publisher.publish(vel)
            rate.sleep()
        else:
            vel.angular.z = (-0.3)/r_min
            velocity_publisher.publish(vel)
            rate.sleep()
    else:
        vel.angular.z = 0
        velocity_publisher.publish(vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        
        rospy.init_node('smoothmover', anonymous=True)

        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        rate = rospy.Rate(10)
        pose_subscriber = rospy.Subscriber('/scan', LaserScan, Callback) 
        time.sleep(2)
        print(len(ranges))
        rospy.spin()
        

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")