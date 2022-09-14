#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from nav_msgs.msg import Odometry

def pose_callback(pose_data):
    global position_x, position_y, orientation
    position_x = pose_data.pose.pose.position.x
    position_y = pose_data.pose.pose.position.y
    orientation_quat = pose_data.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    orientation = tf.transformations.euler_from_quaternion(orientation_list)[2] * (180/math.pi)

def laser_callback(scan_data):
    global position_x, position_y, orientation
    r = list(scan_data.ranges)
    rl = r[0:20]
    rr = r[340:360]
    rt = rl+rr
    goal_direction = math.atan2(translation[1], translation[0])
    if goal_direction >= 0:    
        goal_degree = (goal_direction * (180/math.pi)) - 180
    else:
        goal_degree = (goal_direction * (180/math.pi)) + 180
    degree_diff = (goal_degree - orientation)
    goal_distance = r[int(degree_diff)]
    if (0 <= r.index(goal_distance) <= 15):
        goal_direction_range = r[0:r.index(goal_distance)+15] + r[345+r.index(goal_distance):360]
    elif (345 <= r.index(goal_distance) <= 359):
        goal_direction_range = r[r.index(goal_distance)-15:360] + r[0:r.index(goal_distance)-344]
    else:
        goal_direction_range = r[r.index(goal_distance)-15:r.index(goal_distance)+15]
    
    goal_direction_distance = min(goal_direction_range)

    print('min distance', min(r), 'goal range distance: ', goal_direction_distance)


    if (goal_direction_distance > 0.35):
        if 0 <= r.index(goal_distance) <=2 or 357 <= r.index(goal_distance) <= 359:
            go_straight()
        else:
            if 3 <= r.index(goal_distance) <= 180:
                turn_left()
            elif 181 <= r.index(goal_distance) <= 356:
                turn_right()
        
    else:
        follow_wall(r)

    if (position_x - 2)**2 + position_y**2 < 0.001:
        velocity_message.linear.x = 0
        velocity_message.angular.z = 0
        turtle_velocity.publish(velocity_message)
    
    print('min index: ',r.index(min(r)), 'goal index', r.index(goal_distance))
    print('forward distance', min(rt))
    rate.sleep()

def follow_wall(range_list):
    range_list_l = range_list[0:20]
    range_list_r = range_list[340:360]
    range_list_t = range_list_l + range_list_r
    
    if (0 <= range_list.index(min(range_list)) < 130):
        if (min(range_list_t) > 0.35):
            go_straight()
        else:
            turn_right()        
    if(230 <= range_list.index(min(range_list)) < 360):
        if (min(range_list_t) > 0.35):
            go_straight()
        else:
            turn_left()
    if (180 <= range_list.index(min(range_list)) < 230):
        turn_right()
    if (130 <= range_list.index(min(range_list)) <180):
        turn_left()
    

def go_straight(speed = 0.1):
    velocity_message.linear.x = speed
    velocity_message.angular.z = 0
    turtle_velocity.publish(velocity_message)

def turn_left(speed = 0.3):
    velocity_message.linear.x = 0
    velocity_message.angular.z = speed
    turtle_velocity.publish(velocity_message)

def turn_right(speed = 0.3):
    velocity_message.linear.x = 0
    velocity_message.angular.z = -speed
    turtle_velocity.publish(velocity_message)


if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    transform_listener = tf.TransformListener()
    turtle_scan = rospy.Subscriber('/scan', LaserScan, laser_callback)
    turtle_odom = rospy.Subscriber('/odom', Odometry, pose_callback)
    turtle_velocity = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10.0)
    velocity_message = Twist()

    while not rospy.is_shutdown():
        try:
            (translation,rotation) = transform_listener.lookupTransform('goal_turtle_frame', 'world', rospy.Time(0))
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        

        