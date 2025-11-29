#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("odom_to_pose")

    in_topic  = rospy.get_param("~input_topic", "/odom")
    out_topic = rospy.get_param("~output_topic", "/ground_truth/pose")

    # Publisher lives in enclosing scope for the callback
    pub = rospy.Publisher(out_topic, PoseStamped, queue_size=10)

    def cb(msg):
        ps = PoseStamped()
        ps.header = msg.header          # copy stamp + frame_id
        ps.pose   = msg.pose.pose       # copy pose
        pub.publish(ps)

    rospy.Subscriber(in_topic, Odometry, cb, queue_size=10)

    rospy.loginfo("odom_to_pose: %s -> %s", in_topic, out_topic)
    rospy.spin()


if __name__ == "__main__":
    main()
