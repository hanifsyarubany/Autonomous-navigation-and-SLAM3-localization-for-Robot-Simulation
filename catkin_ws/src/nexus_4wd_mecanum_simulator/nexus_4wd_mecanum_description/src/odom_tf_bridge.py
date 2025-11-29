#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf

br = tf.TransformBroadcaster()

def cb(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    # Use odometry timestamp; parent is the fixed 'odom' frame
    br.sendTransform(
        (p.x, p.y, p.z),
        (q.x, q.y, q.z, q.w),
        msg.header.stamp,
        'base_footprint',   # child
        'odom'              # parent
    )

if __name__ == '__main__':
    rospy.init_node('odom_tf_bridge')
    rospy.Subscriber('odom', Odometry, cb, queue_size=50)
    rospy.spin()
