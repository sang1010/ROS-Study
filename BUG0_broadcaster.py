#!/usr/bin/env python  
import rospy
import tf
from nav_msgs.msg import Odometry

def odom_callback(odom_msg):
    transform_broadcaster = tf.TransformBroadcaster()

    rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)

    translation_vector = (2-(odom_msg.pose.pose.position.x), -(odom_msg.pose.pose.position.y), 0)

    current_time = rospy.Time.now()

    transform_broadcaster.sendTransform(translation_vector, rotation_quaternion,
        current_time, "goal_turtle_frame", "world")

    print('broad success')

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()