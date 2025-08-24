#!/usr/bin/env python3
import os, math, sys, rospy
# ----- choose backend automatically -----
if not os.environ.get("DISPLAY"):      # no GUI
    import matplotlib
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# -------- rest identical --------------
x_data, y_data = [], []

def odom_cb(msg):
    x_data.append(msg.pose.pose.position.x)
    y_data.append(msg.pose.pose.position.y)

def get_float(prompt):
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("  ❗ enter a number.")

def main():
    try:
        n_seg = int(input("How many velocity sets? (1-5) : "))
    except ValueError:
        sys.exit("Bad number – quitting.")

    segments = []
    for i in range(n_seg):
        print(f"\nSegment {i+1}/{n_seg}")
        v = get_float("  linear  v  [m/s] : ")
        w = get_float("  angular w [rad/s]: ")
        d = get_float("  duration  [s]    : ")
        segments.append((v, w, d))

    rospy.init_node("segment_runner")
    pub = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/RosAria/pose", Odometry, odom_cb)

    rate = rospy.Rate(10)
    rospy.sleep(1.0)

    cmd = Twist()
    for v, w, dur in segments:
        cmd.linear.x, cmd.angular.z = v, w
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < dur and not rospy.is_shutdown():
            pub.publish(cmd)
            rate.sleep()

    pub.publish(Twist())
    rospy.sleep(0.2)

    # ---------- plot ----------
    plt.figure(figsize=(5,5))
    plt.plot(x_data, y_data, "-b")
    plt.xlabel("X [m]");  plt.ylabel("Y [m]")
    plt.title("Pioneer Trajectory"); plt.axis("equal"); plt.grid()

    if os.environ.get("DISPLAY"):
        plt.show()
    else:
        outfile = "trajectory.png"
        plt.savefig(outfile, dpi=150)
        print(f"Trajectory saved to {outfile} – copy it to your laptop.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

