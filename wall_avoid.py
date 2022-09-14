#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time


def callback(scan_data):
    r = list(scan_data.ranges)
    rg1 = r[0:10]
    rg2 = r[350:360]
    rg = rg1 + rg2
    # print(rg)
    move(rg)
    print(len(r))
    print(min(rg))

def move(range_list):
    if(min(range_list[0:10]) <= 0.6):
        velocity.linear.x = 0.0
        velocity.angular.z = 1
        vel.publish(velocity)
        print(velocity)
    
    if(min(range_list[10:20]) <= 0.6):
        velocity.linear.x = 0.0
        velocity.angular.z = -1
        vel.publish(velocity)
        print(velocity)

    if(min(range_list) > 3.0):
        velocity.linear.x = 1
        velocity.angular.z = 0.0
        vel.publish(velocity)
        print(velocity)

    if(0.6 < min(range_list) <= 3.0):
        if(velocity.linear.x == 0.0 and velocity.angular.z == 0.0):
            velocity.linear.x = 1
            velocity.angular.z = 0
            vel.publish(velocity)
            print(velocity)
        else:
            pass
    rate.sleep()                  

if __name__ == '__main__':
    try:
        rospy.init_node('mover', anonymous=True)

        velocity = Twist()
        vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(200)

        subs = rospy.Subscriber('/scan', LaserScan, callback)
        time.sleep(2)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass