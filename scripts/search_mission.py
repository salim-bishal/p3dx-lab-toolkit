#!/usr/bin/env python3

import rospy
import math
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Global variables for odometry data
x = 0.0
y = 0.0
yaw = 0.0

# Initial (start) pose
initial_x = None
initial_y = None
initial_yaw = None
has_initialized = False

# State machine (7 steps)
state = 0

# Distances / angles
FORWARD_DIST = 2.0          # forward/back 2m
TURN_ANGLE_DEG = 90.0       # 90° turns
TURN_TOLERANCE_DEG = 2.0
DIST_TOLERANCE = 0.05

# Speeds
LIN_SPEED = 0.2
ANG_SPEED = 0.3

# Publisher (will be set in main)
pub = None

def odom_callback(msg):
    """
    Subscribes to /RosAria/pose (type nav_msgs/Odometry).
    Extract x, y, yaw from the odometry data.
    """
    global x, y, yaw
    global initial_x, initial_y, initial_yaw
    global has_initialized

    pose = msg.pose.pose

    # Position
    x = pose.position.x
    y = pose.position.y

    # Orientation (quaternion -> euler for yaw)
    orientation_q = pose.orientation
    quaternion = (
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]  # in radians

    # On first callback, store initial pose
    if not has_initialized:
        initial_x = x
        initial_y = y
        initial_yaw = yaw
        has_initialized = True
        rospy.loginfo("Initial pose set: x=%.2f, y=%.2f, yaw=%.2f deg",
                      x, y, math.degrees(yaw))

def distance_from_initial():
    dx = x - initial_x
    dy = y - initial_y
    return math.sqrt(dx*dx + dy*dy)

def distance_to(tx, ty):
    dx = x - tx
    dy = y - ty
    return math.sqrt(dx*dx + dy*dy)

def wrap_angle(angle):
    """
    Normalize angle to [-pi, pi].
    """
    return math.atan2(math.sin(angle), math.cos(angle))

def is_angle_reached(target_angle):
    """
    Check if current yaw is within TURN_TOLERANCE_DEG of target_angle (in radians).
    """
    current_yaw_deg = math.degrees(wrap_angle(yaw))
    target_yaw_deg = math.degrees(wrap_angle(target_angle))
    diff = wrap_angle(math.radians(target_yaw_deg - current_yaw_deg))
    diff_deg = math.degrees(diff)
    return abs(diff_deg) < TURN_TOLERANCE_DEG

def main():
    global pub
    global state, has_initialized

    rospy.init_node('search_mission', anonymous=True)
    rospy.loginfo("Search mission (RosAria) started.")

    # Publish Twist commands to /RosAria/cmd_vel
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

    # Subscribe to /RosAria/pose for odometry
    rospy.Subscriber('/RosAria/pose', Odometry, odom_callback)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if not has_initialized:
            # Wait until we have an initial pose
            rate.sleep()
            continue

        cmd = Twist()

        if state == 0:
            # Move forward 2m
            if distance_from_initial() < FORWARD_DIST:
                cmd.linear.x = LIN_SPEED
            else:
                cmd.linear.x = 0.0
                state += 1
                rospy.loginfo("Transition to state %d", state)

        elif state == 1:
            # Turn right 90°
            target_yaw = wrap_angle(initial_yaw - math.radians(TURN_ANGLE_DEG))
            if not is_angle_reached(target_yaw):
                cmd.angular.z = -ANG_SPEED
            else:
                cmd.angular.z = 0.0
                state += 1
                rospy.loginfo("Transition to state %d", state)

        elif state == 2:
            # Turn left 90° (back to initial heading)
            if not is_angle_reached(initial_yaw):
                cmd.angular.z = ANG_SPEED
            else:
                cmd.angular.z = 0.0
                state += 1
                rospy.loginfo("Transition to state %d", state)

        elif state == 3:
            # Turn left 90° again
            target_yaw = wrap_angle(initial_yaw + math.radians(TURN_ANGLE_DEG))
            if not is_angle_reached(target_yaw):
                cmd.angular.z = ANG_SPEED
            else:
                cmd.angular.z = 0.0
                state += 1
                rospy.loginfo("Transition to state %d", state)

        elif state == 4:
            # Turn right 90° (back to initial heading again)
            if not is_angle_reached(initial_yaw):
                cmd.angular.z = -ANG_SPEED
            else:
                cmd.angular.z = 0.0
                state += 1
                rospy.loginfo("Transition to state %d", state)

        elif state == 5:
            # Return to start (2m backward)
            dist_to_start = distance_to(initial_x, initial_y)
            if dist_to_start > 0.2:
                cmd.linear.x = -LIN_SPEED
            else:
                cmd.linear.x = 0.0
                state += 1
                rospy.loginfo("Transition to state %d", state)

        elif state == 6:
            # Stop and exit
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            pub.publish(cmd)
            rospy.loginfo("Mission complete. Stopping.")
            break

        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

