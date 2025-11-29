#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist

def publish_twist(pub, vx=0.0, wz=0.0):
    msg = Twist()
    msg.linear.x = vx
    msg.angular.z = wz
    pub.publish(msg)

def hold(pub, vx, wz, duration_s, rate_hz=20.0):
    rate = rospy.Rate(rate_hz)
    t_end = rospy.Time.now() + rospy.Duration(duration_s)
    while not rospy.is_shutdown() and rospy.Time.now() < t_end:
        publish_twist(pub, vx, wz)
        rate.sleep()

def wait_for_sub(pub, timeout_s=5.0):
    deadline = rospy.Time.now() + rospy.Duration(timeout_s)
    r = rospy.Rate(20)
    while not rospy.is_shutdown() and pub.get_num_connections() == 0 and rospy.Time.now() < deadline:
        r.sleep()

def run():
    rospy.init_node("l_motion_loop")

    # --- Parameters (all overridable via <param .../> in your launch) ---
    topic       = rospy.get_param("~topic", "/r2d2_diff_drive_controller/cmd_vel")
    v           = float(rospy.get_param("~v", 0.2))          # forward speed (m/s)
    w           = float(rospy.get_param("~w", 0.5))          # yaw rate (rad/s)
    dist1       = float(rospy.get_param("~dist1", 5.0))      # first leg length (m)
    dist2       = float(rospy.get_param("~dist2", 1.0))      # second leg length (m)
    angle_deg   = float(rospy.get_param("~angle_deg", 90.0)) # corner angle (deg)
    turn_left   = bool(rospy.get_param("~turn_left", True))  # True = left L, False = right L
    settle      = float(rospy.get_param("~settle", 0.3))     # pause between phases (s)
    rate_hz     = float(rospy.get_param("~rate_hz", 20.0))   # publish rate during holds
    dwell_end   = float(rospy.get_param("~dwell_end", 0.5))  # pause at the far corner and at start

    # Pre-compute durations
    ang = abs(math.radians(angle_deg))
    if v <= 0.0 or w <= 0.0:
        rospy.logwarn("Params v and w must be > 0. Using defaults v=0.2, w=0.5.")
        v = max(v, 0.2)
        w = max(w, 0.5)

    t1 = dist1 / v   # forward first leg
    t2 = dist2 / v   # forward second leg
    t_turn = ang / w

    # Turn signs
    wz_fwd =  +w if turn_left else -w   # forward corner
    wz_rev =  -w if turn_left else +w   # reverse corner (undo heading)

    pub = rospy.Publisher(topic, Twist, queue_size=1)
    wait_for_sub(pub)

    rate = rospy.Rate(rate_hz)
    rospy.loginfo("Starting continuous L-pattern loop on %s", topic)

    while not rospy.is_shutdown():
        # -------- FORWARD L --------
        # leg 1
        hold(pub, +v, 0.0, t1, rate_hz)
        hold(pub, 0.0, 0.0, settle, rate_hz)

        # corner turn
        hold(pub, 0.0, wz_fwd, t_turn, rate_hz)
        hold(pub, 0.0, 0.0, settle, rate_hz)

        # leg 2
        hold(pub, +v, 0.0, t2, rate_hz)
        hold(pub, 0.0, 0.0, dwell_end, rate_hz)

        # -------- REVERSE L (back to start) --------
        # leg 2 back
        hold(pub, -v, 0.0, t2, rate_hz)
        hold(pub, 0.0, 0.0, settle, rate_hz)

        # reverse corner turn (undo the angle)
        hold(pub, 0.0, wz_rev, t_turn, rate_hz)
        hold(pub, 0.0, 0.0, settle, rate_hz)

        # leg 1 back
        hold(pub, -v, 0.0, t1, rate_hz)
        hold(pub, 0.0, 0.0, dwell_end, rate_hz)

    # final stop
    publish_twist(pub, 0.0, 0.0)

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
