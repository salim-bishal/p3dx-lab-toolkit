#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time
import math

# Initialize lists for plotting
time_data = []
desired_linear_vel = []
desired_angular_vel = []
actual_linear_vel = []
actual_angular_vel = []

desired_path_x = []
desired_path_y = []
actual_path_x = []
actual_path_y = []

# Robot parameters
RADIUS = 0.4  # Desired circle radius (meters)
LINEAR_SPEED = 0.2  # Desired linear speed (m/s)
ANGULAR_SPEED = LINEAR_SPEED / RADIUS  # Derived angular speed (rad/s)

# Variables to track position
actual_x = 0
actual_y = 0
theta = 0  # Orientation

# Callback function to update actual velocities
def odom_callback(msg):
    global actual_x, actual_y, theta

    # Append actual velocities
    actual_linear_vel.append(msg.linear.x)
    actual_angular_vel.append(msg.angular.z)

    # Update robot position based on velocities
    dt = 0.1  # Assuming 10 Hz update rate
    theta += msg.angular.z * dt
    actual_x += msg.linear.x * math.cos(theta) * dt
    actual_y += msg.linear.x * math.sin(theta) * dt

    # Append actual position
    actual_path_x.append(actual_x)
    actual_path_y.append(actual_y)

def move_in_circle():
    # Initialize ROS node
    rospy.init_node('circle_movement', anonymous=True)
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/RosAria/cmd_vel', Twist, odom_callback)
    rate = rospy.Rate(10)  # 10 Hz

    start_time = time.time()
    twist = Twist()
    twist.linear.x = LINEAR_SPEED
    twist.angular.z = ANGULAR_SPEED

    rospy.loginfo("Moving the robot in a circle...")

    while not rospy.is_shutdown():
        elapsed_time = time.time() - start_time

        # Stop the robot after 30 seconds
        if elapsed_time > 30:
            twist.linear.x = 0
            twist.angular.z = 0
            pub.publish(twist)
            break

        # Publish desired velocities
        pub.publish(twist)

        # Record desired velocities and positions
        time_data.append(elapsed_time)
        desired_linear_vel.append(LINEAR_SPEED)
        desired_angular_vel.append(ANGULAR_SPEED)
        desired_path_x.append(RADIUS * math.cos(ANGULAR_SPEED * elapsed_time))
        desired_path_y.append(RADIUS * math.sin(ANGULAR_SPEED * elapsed_time))

        rate.sleep()

    # Plot data after the movement
    plot_velocity()
    plot_path()

def plot_velocity():
    # Plot linear and angular velocity
    plt.figure(figsize=(8, 6))

    # Linear velocity
    plt.plot(time_data, desired_linear_vel, label="Desired Linear Velocity", linestyle='--', color='blue')
    plt.plot(time_data[:len(actual_linear_vel)], actual_linear_vel, label="Actual Linear Velocity", color='green')
    plt.xlabel("Time (s)")
    plt.ylabel("Linear Velocity (m/s)")
    plt.title("Linear Velocity vs. Time")
    plt.legend()
    plt.grid()
    plt.savefig("desired_linear_velocity.png")
    rospy.loginfo("Saved desired linear velocity plot as desired_linear_velocity.png")
    plt.clf()

    # Angular velocity
    plt.plot(time_data, desired_angular_vel, label="Desired Angular Velocity", linestyle='--', color='red')
    plt.plot(time_data[:len(actual_angular_vel)], actual_angular_vel, label="Actual Angular Velocity", color='orange')
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity (rad/s)")
    plt.title("Angular Velocity vs. Time")
    plt.legend()
    plt.grid()
    plt.savefig("desired_angular_velocity.png")
    rospy.loginfo("Saved desired angular velocity plot as desired_angular_velocity.png")
    plt.clf()

def plot_path():
    # Plot desired and actual paths
    plt.figure(figsize=(8, 6))
    plt.plot(desired_path_x, desired_path_y, label="Desired Path", linestyle='--', color='purple')
    plt.plot(actual_path_x, actual_path_y, label="Actual Path", color='brown')
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Desired vs. Actual Path")
    plt.legend()
    plt.grid()
    plt.savefig("desired_actual_path.png")
    rospy.loginfo("Saved desired and actual path plot as desired_actual_path.png")
    plt.clf()

if __name__ == '__main__':
    try:
        move_in_circle()
    except rospy.ROSInterruptException:
        pass

