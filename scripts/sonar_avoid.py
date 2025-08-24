#!/usr/bin/env python3
"""
sonar_avoid.py
------------------------------------------
ROS node for Pioneer 3-DX + RosAria sonar ring.

• Publishes  /RosAria/cmd_vel
• Subscribes /RosAria/sonar_pointcloud2

Logic
  → Drive forward (FWD_V) until any sonar point in ±45° front arc
    is closer than TRIP_DIST.
  → Turn left (TURN_RATE) until the nearest front distance exceeds
    RELEASE_DIST, then resume forward.

Terminal messages:
  - Start-up prompt to "stand in front"
  - "⚠️ Obstacle detected …"
  - "✅ Cleared obstacle … stand near again …"
"""

import math
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist

# ---- parameters ----
TRIP_DIST     = 0.80   # m  start turning closer than this
RELEASE_DIST  = 1.00   # m  resume forward when clear
FWD_V         = 0.15   # m/s forward speed
TURN_RATE     = 0.5    # rad/s left turn
FRONT_ARC_DEG = 45     # degrees to either side of x-axis

# ---- globals ----
nearest_front = 999.0
turning       = False

def pc2_cb(msg: PointCloud2):
    """Update nearest_front with closest point in ±front arc."""
    global nearest_front
    nearest_front = 999.0
    for x, y, _ in pc2.read_points(msg, ("x", "y", "z"), skip_nans=True):
        if x > 0:                         # ignore everything behind robot
            ang_deg = math.degrees(math.atan2(y, x))
            if abs(ang_deg) < FRONT_ARC_DEG:
                dist = math.hypot(x, y)
                nearest_front = min(nearest_front, dist)

def main():
    global turning
    rospy.init_node("sonar_front_avoid_log")
    pub = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=5)
    rospy.Subscriber("/RosAria/sonar_pointcloud2", PointCloud2, pc2_cb)

    rate = rospy.Rate(10)
    rospy.sleep(1.0)          # wait a moment for first sonar cloud

    rospy.loginfo("📢 Node ready — stand in front of the robot to trigger avoidance.")

    while not rospy.is_shutdown():
        cmd = Twist()

        if not turning:
            if nearest_front < TRIP_DIST:
                turning = True
                rospy.loginfo(
                    "⚠️ Obstacle detected at %.2f m: starting to turn left",
                    nearest_front)
            else:
                cmd.linear.x = FWD_V
        else:
            cmd.angular.z = TURN_RATE
            if nearest_front > RELEASE_DIST:
                turning = False
                rospy.loginfo(
                    "✅ Cleared obstacle: resuming forward. "
                    "You can stand near again to trigger another run.")

        pub.publish(cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

