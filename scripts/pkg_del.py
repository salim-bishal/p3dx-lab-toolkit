#!/usr/bin/env python3
"""
Interactive waypoint runner:
• User repeatedly enters (linear v, angular w, duration) triples.
• Script executes each triple in order.
• After each batch it asks if the robot has arrived.
• Logs odometry and saves / shows a trajectory plot.
"""

import os, math, sys, time, rospy, tf
import matplotlib
# choose non-GUI backend if no X-server
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ------------- globals -------------
x_log, y_log = [], []
latest_speed = 0.0

def odom_cb(msg):
    global latest_speed
    x_log.append(msg.pose.pose.position.x)
    y_log.append(msg.pose.pose.position.y)
    latest_speed = msg.twist.twist.linear.x

def ask_float(label):
    while True:
        try:
            return float(input(label))
        except ValueError:
            print("  ❗  Please enter a number.")

def get_segment(idx):
    print(f"Segment {idx}:")
    v = ask_float(" Enter linear velocity v [m/s]  : ")
    w = ask_float(" Enter angular velocity w [rad/s]: ")
    d = ask_float(" Enter duration  [s]    : ")
    return v, w, d

def execute_list(pub, segments, rate):
    cmd = Twist()
    for v, w, dur in segments:
        cmd.linear.x, cmd.angular.z = v, w
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < dur and not rospy.is_shutdown():
            pub.publish(cmd); rate.sleep()
    # stop briefly between batches
    pub.publish(Twist()); rospy.sleep(0.3)

def main():
    rospy.init_node("interactive_runner")
    pub  = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/RosAria/pose", Odometry, odom_cb)
    rate = rospy.Rate(10)
    rospy.sleep(1.0)               # wait for odom

    t_start = time.time()

    # -------- OUTBOUND --------
    batch = 1
    while True:
        print(f"\n--- Outbound batch #{batch} ---")
        segs = [get_segment(1)]
        execute_list(pub, segs, rate)
        if input("Reached destination/Point B? (type y for yes/n for no): ").strip().lower() == "y":
            break
        batch += 1

    # -------- RETURN --------
    print("\n=== Return trip ===")
    batch = 1
    while True:
        print(f"\nReturn batch #{batch}")
        segs = [get_segment(1)]
        execute_list(pub, segs, rate)
        if input("Back at start/Point A? (type y for yes/ n for no): ").strip().lower() == "y":
            break
        batch += 1

    # -------- wrap-up --------
    total_wall = time.time() - t_start
    pub.publish(Twist())           # ensure stop
    rospy.sleep(0.2)

    # plot
    plt.figure(figsize=(5,5))
    plt.plot(x_log, y_log, "-b")
    plt.xlabel("X [m]"); plt.ylabel("Y [m]")
    plt.title("Robot trajectory"); plt.axis("equal"); plt.grid(True)
    if os.environ.get("DISPLAY"):
        plt.show()
    else:
        plt.savefig("trajectory.png", dpi=150)
        print("\nTrajectory saved as trajectory.png.")

    print(f"\nTotal elapsed time (motion + pauses): {total_wall:.1f} s")

if __name__ == "__main__":
    try:
        main()       
    except rospy.ROSInterruptException:
        pass

