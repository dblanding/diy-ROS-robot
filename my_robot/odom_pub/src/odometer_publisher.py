#!/usr/bin/env python3
"""
ROS Odometry node

Subscribes to topics:
    /rigt_ticks
    /left_ticks

Publishes on topic /odom and broadcasts tf data

Adapted from https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
a python version of http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
"""

from math import asin, sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Int32, Float32

rospy.init_node('odometry_publisher', anonymous=True)

TICKS_PER_METER = rospy.get_param('ROBOT_TICKS_PER_METER')
TRACK_WIDTH = rospy.get_param('ROBOT_TRACK_WIDTH')

# Listen for encoder data
left_ticks = 0
right_ticks = 0

def left_tick_callback(msg):
    global left_ticks
    left_ticks = msg.data

def right_tick_callback(msg):
    global right_ticks
    right_ticks = msg.data

def left_tick_listener():
    rospy.Subscriber("/left_ticks", Int32, left_tick_callback)

def right_tick_listener():
    rospy.Subscriber("/right_ticks", Int32, right_tick_callback)

# Initial values for robot pose
x = 0.0  # x position
y = 0.0  # Y position
th = 0.0  # theta Z orientaion 

prev_left_ticks = 0
prev_right_ticks = 0

last_time = rospy.Time.now()

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

# Start listening
left_tick_listener()
right_tick_listener()

r = rospy.Rate(10.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()

    # Calculate the distance traveled since the last cycle (meters)
    left_dist = (left_ticks - prev_left_ticks) / TICKS_PER_METER
    prev_left_ticks = left_ticks
    right_dist = (right_ticks - prev_right_ticks) / TICKS_PER_METER
    prev_right_ticks = right_ticks

    # Update odometry information since previous cycle
    cycle_dist = (left_dist + right_dist) / 2
    cycle_diff = right_dist - left_dist

    try:
        cycle_angle = asin(cycle_diff / TRACK_WIDTH)
    except ValueError:  # Occasionally get math domain error
        cycle_angle = 0
        print("math domain error")

    avg_angle = cycle_angle / 2 + th

    # Calculate the position and orientation
    delta_x = cos(avg_angle) * cycle_dist
    x += delta_x
    delta_y = sin(avg_angle) * cycle_dist
    y += delta_y
    delta_th = cycle_angle
    th += delta_th
    
    # Calculate the velocities
    vx = delta_x / dt
    vy = delta_y / dt
    vth = delta_th / dt

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    '''
    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_footprint",
        "odom"
    )
    '''
    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the values of covariance on the diagonal
    odom.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # set the values of covariance on the diagonal
    odom.twist.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
