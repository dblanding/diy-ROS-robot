#! /usr/bin/env python3
"""
Utility for discovering motor characteristics:
    relation between tick rate & velocity (m/s)

Publish on topic:
    /cmd_vel
    
Subscribe to topics:
    /right_ticks
    /left_ticks

Time robot while driving DIST at VEL.
Measure how far robot moved.
Check time and dist to see if VEL was accurate.

Use data to fill in best-fit relation between tick_rate & mtr_spd
"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32, String
import sys
import time

VEL = 0.2  # meters/sec
DIST = 1.0  # meters
TICKS_PER_METER = 3239
TIMEOUT = 10  # seconds to timeout

left_ticks = 0
right_ticks = 0

def left_listener():
    rospy.Subscriber("/left_ticks", Int32, left_callback)

def left_callback(msg):
    global left_ticks
    left_ticks = msg.data

def right_listener():
    rospy.Subscriber("/right_ticks", Int32, right_callback)

def right_callback(msg):
    global right_ticks
    right_ticks = msg.data

def stopOnShutdown():
    rospy.loginfo("Got Ctrl-C")
    rospy.loginfo("Shutting down")
    sys.exit(0)

if __name__ == '__main__':

    # Listen to /left_ticks & /right_ticks topics
    left_listener()
    right_listener()

    # Publish cmd_vel to drive at VEL (m/s) for SECONDS (s)
    vel_msg = Twist()
    vel_msg.linear.x = VEL

    robot_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.init_node('tester', anonymous=True)
    rate = rospy.Rate(10)
    #rospy.on_shutdown(stopOnShutdown)

    # wait for inital reading of encoder ticks
    while  right_ticks == 0:
        pass
    start_right_ticks = right_ticks

    #rospy.loginfo(start_left_ticks, start_right_ticks)

    start_time = rospy.Time.now().to_sec()
    done = False
    dist = 0
    robot_vel.publish(vel_msg)
    print("Entering Loop")
    while not done:
        rate.sleep()
        total_ticks = right_ticks - start_right_ticks
        dist = total_ticks / TICKS_PER_METER
        if dist >= DIST:
            done = True
            print(f"Stopped at {right_ticks - start_right_ticks} ticks")
        time_lapsed = rospy.Time.now().to_sec()-start_time
        if time_lapsed > TIMEOUT:
            done = True
            print("Timed out")

    vel_msg.linear.x = 0
    robot_vel.publish(vel_msg)
    elapsed_time = rospy.Time.now().to_sec() - start_time
    #total_left_ticks = left_ticks - start_left_ticks
    time.sleep(0.5)  # Allow time to coast to stop
    total_right_ticks = right_ticks - start_right_ticks
    rospy.loginfo(f"Drove at {VEL} meters/sec for {elapsed_time} seconds")
    #rospy.loginfo(f"Left_ticks = {total_left_ticks}")
    rospy.loginfo(f"Coasted to = {total_right_ticks} ticks")
    rospy.loginfo(f"elapsed time = {elapsed_time}")
    rospy.loginfo(f"Calculated velocity = {1 / elapsed_time} m/sec")
    rospy.loginfo(f"Distance travelled = {(right_ticks-start_right_ticks)/TICKS_PER_METER}")
    #rospy.loginfo(f"Avg_ticks = {(total_right_ticks+total_left_ticks)/2}")
