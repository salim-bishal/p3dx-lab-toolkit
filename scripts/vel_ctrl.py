#!/usr/bin/env python3
"""
run_p3dx_interactive.py
---------------------------------
Interactive tele-run script for a Pioneer 3-DX (RosAria driver).

• Asks the operator for linear & angular velocity set-points.
• Asks for run-duration (seconds). 0 = run until Ctrl+C.
• Publishes Twist on /RosAria/cmd_vel at 10 Hz.
• Subscribes to /RosAria/pose so, when motion stops, it prints the
  *latest measured* linear.x and angular.z from odometry.
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

latest_lin = 0.0
latest_ang = 0.0

def odom_cb(msg):
    global latest_lin, latest_ang
    latest_lin = msg.twist.twist.linear.x
    latest_ang = msg.twist.twist.angular.z

def main():
    rospy.init_node("p3dx_user_run", anonymous=True)
    pub = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/RosAria/pose", Odometry, odom_cb)

    # ---------- user inputs ----------
    try:
        v_input = float(input("Linear velocity [m/s]  (e.g. 0.2) : "))
        w_input = float(input("Angular velocity [rad/s] (e.g. 0.0) : "))
        duration = float(input("Run time [s] (0 = until Ctrl+C) : "))
    except ValueError:
        print("❗ Invalid number — exiting.")
        return

    cmd = Twist()
    cmd.linear.x  = v_input
    cmd.angular.z = w_input

    rate = rospy.Rate(10)          # 10 Hz publish loop
    start_time = rospy.Time.now()

    print("\n>>> Running...  Press Ctrl+C to stop early.\n")

    try:
        while not rospy.is_shutdown():
            pub.publish(cmd)

            if duration > 0.0:
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed >= duration:
                    break

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        # stop the robot
        pub.publish(Twist())
        rospy.sleep(0.1)

        print("=== Motion stopped ===")
        print(f"Last measured linear.x  : {latest_lin:.3f}  m/s")
        print(f"Last measured angular.z : {latest_ang:.3f}  rad/s")

if __name__ == "__main__":
    main()

