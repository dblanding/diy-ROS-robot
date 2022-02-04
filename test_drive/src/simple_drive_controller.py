#!/usr/bin/env python3
"""
Subscribe to topics:
    /move_base_simple/goal  # message type: PoseStamped
    /robot_pose_ekf/odom_combined  # message type: PoseWithCovarianceStamped

Publish on topic:
    /cmd_vel  # message type: Twist

This node was intended to be a Python version of drive_controller.cpp
from Chapter 12 of Lloyd Brombach's book Practical Robotics in C++.
However it is not a 1:1 version. Instead, it allows using RVIZ to
operate the robot. Like Lloyd's program, it doesn't avoid obstacles.
1. It turns and aims toward the goal
2. then drives to the goal
3. then rotates to the goal orientation.

Author: Doug Blanding (dblanding@gmail.com)
12/17/21
"""

import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

V_TURN_IN_PLACE = 0.5  # Angular vel for turning in place (rad/s)
AIM = 0.07  # Angular threshold at which turn in place is stopped (rad)
V_CRUISE = 0.3  # Translational cruising vel (m/s)
V_CREEP = 0.1  # Translatioinal creeping vel (m/s)
CLOSE = 0.1  # Dist threshold to transition from V_CRUISE to V_CREEP (m)
AT_GOAL = 0.04  # Dist threshold to be considered to be at goal (m)

new_goal = False
pose = (0.0, 0.0, 0.0)  # (x_pose, y_pose, heading_pose)
goal = (0.0, 0.0, 0.0)  # (x_goal, y_goal, heading_goal)


def goal_listener():
    """Listen to /move_base_simple/goal topic. Send msg to goal_callback. """
    rospy.Subscriber("/move_base_simple/goal",
                     PoseStamped, goal_callback)


def goal_callback(msg):
    """Extract pose position and orientation from PoseStamped msg."""
    global goal, new_goal
    x = msg.pose.position.x  # meters
    y = msg.pose.position.y  # meters
    quaternion = msg.pose.orientation
    roll, pitch, yaw = get_rotation(quaternion)
    new_goal = True
    goal = (x, y, yaw)


def pose_listener():
    """
    Listen to /robot_pose_ekf/odom_combined topic. Send msg to pose_callback.
    """
    rospy.Subscriber("/robot_pose_ekf/odom_combined",
                     PoseWithCovarianceStamped, pose_callback)


def pose_callback(msg):
    """
    Extract pose position and orientation from PoseWithCovarianceStamped msg.
    """
    global pose
    x_position = msg.pose.pose.position.x  # meters
    y_position = msg.pose.pose.position.y  # meters
    quaternion = msg.pose.pose.orientation
    roll, pitch, yaw = get_rotation(quaternion)
    pose = (x_position, y_position, yaw)


def get_rotation(quaternion):
    """Convert quaternion to euler angles."""
    quat_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    roll, pitch, yaw = euler_from_quaternion(quat_list)
    return roll, pitch, yaw


def get_turn_angle():
    """Calculate turn angle needed to put goal straight ahead."""
    _, _, heading_pose = pose
    _, _, angle_rel = goal_wrt_pose()
    angle_to_turn = angle_rel - heading_pose
    return angle_to_turn


def dist_to_goal():
    """Calculate distance to goal (m)"""
    x_rel, y_rel, _ = goal_wrt_pose()
    dist = math.sqrt(x_rel**2 + y_rel**2)
    return dist


def goal_wrt_pose():
    """Calculate relative dist and angle of goal w/r/t robot."""
    x_pose, y_pose, heading_pose = pose
    x_goal, y_goal, heading_goal = goal
    x_rel = x_goal - x_pose
    y_rel = y_goal - y_pose
    angle_rel = math.atan2(y_rel, x_rel)
    return x_rel, y_rel, angle_rel


def turn_in_place():
    """Turn in place so that goal is straight ahead of robot."""
    cmd_vel = Twist()
    angle_to_turn = get_turn_angle()
    rospy.loginfo(f"Angle to turn is {angle_to_turn:.4f} rad")
    while abs(angle_to_turn) > AIM:
        if angle_to_turn > 0:
            cmd_vel.angular.z = V_TURN_IN_PLACE
        else:
            cmd_vel.angular.z = -V_TURN_IN_PLACE
        drive.publish(cmd_vel)
        rate.sleep()
        angle_to_turn = get_turn_angle()
    cmd_vel.angular.z = 0.0
    drive.publish(cmd_vel)
    rospy.loginfo(f"Finished Turn. Angle to Goal is {angle_to_turn:.4f} rad")


def drive_to_goal():
    """Drive straight to goal and stop when one of the following occurs:

    1. Robot arrives within threshold value of goal or
    2. Dist to goal begins to increase
    """
    global new_goal
    cmd_vel = Twist()
    goal_dist = dist_to_goal()
    prev_goal_dist = goal_dist
    delta_goal_dist = 0
    while goal_dist > AT_GOAL and delta_goal_dist < 0.01:
        if goal_dist < CLOSE:
            cmd_vel.linear.x = V_CREEP
        else:
            cmd_vel.linear.x = V_CRUISE
        drive.publish(cmd_vel)
        rate.sleep()
        goal_dist = dist_to_goal()
        delta_goal_dist = goal_dist - prev_goal_dist
        prev_goal_dist = goal_dist
    cmd_vel.linear.x = 0.0
    drive.publish(cmd_vel)
    rospy.loginfo(f"Arrived at goal. Distance to goal is {goal_dist:.2f} m")
    new_goal = False


if __name__ == "__main__":

    rospy.init_node('test_drive', anonymous=True)
    rate = rospy.Rate(10)

    # Listen to goal topic
    goal_listener()

    # Listen to pose topic
    pose_listener()

    # Set up to publish cmd_vel
    cmd_vel = Twist()
    drive = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        if new_goal:
            turn_in_place()
            drive_to_goal()

        rate.sleep()
