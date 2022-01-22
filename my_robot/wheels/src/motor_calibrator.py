#!/usr/bin/env python3
""" Drive wheel motors at a range of speeds (pwm values)
    dwelling for DWELL seconds at each speed
    collect and average measured tick rates at each speed
    and save data to datafile.

Saved data used to populate TRS_curve values for motors.

Decoder class from rotary_encoder.py example at:
https://abyz.me.uk/rpi/pigpio/examples.html
Hook up A and B so that values increase when moving forward.
pigpio daemon must be running. Launch with 'sudo pigpiod'.
"""

import rospy
import pigpio
from rotary_encoder import Decoder

# Save data in datafile
datafile = '/home/ubuntu/catkin_ws/src/my_robot/wheels/data/tr_spd.csv'
DWELL = 4  # seconds to dwell at each speed

# Set up gpio (Broadcom) pin aliases
left_mtr_spd_pin = 17
left_mtr_in1_pin = 27
left_mtr_in2_pin = 22

right_mtr_spd_pin = 11
right_mtr_in1_pin = 10
right_mtr_in2_pin = 9

left_enc_A_pin = 7
left_enc_B_pin = 8

right_enc_A_pin = 23
right_enc_B_pin = 24

L_mode = 'OFF'  # motor mode: 'FWD', 'REV', 'OFF'
R_mode = 'OFF'
    
def set_mtr_spd(pi, L_PWM_val, R_PWM_val, L_mode, R_mode):
    """Drive motors using a PWM signal."""

    # Set motor direction pins appropriately
    if L_mode == 'FWD':
        pi.write(left_mtr_in1_pin, 0)
        pi.write(left_mtr_in2_pin, 1)
    elif L_mode == 'REV':
        pi.write(left_mtr_in1_pin, 1)
        pi.write(left_mtr_in2_pin, 0)
    else:  # Parked
        pi.write(left_mtr_in1_pin, 0)
        pi.write(left_mtr_in2_pin, 0)

    if R_mode == 'FWD':
        pi.write(right_mtr_in1_pin, 0)
        pi.write(right_mtr_in2_pin, 1)
    elif R_mode == 'REV':
        pi.write(right_mtr_in1_pin, 1)
        pi.write(right_mtr_in2_pin, 0)
    else:  # Parked
        pi.write(right_mtr_in1_pin, 0)
        pi.write(right_mtr_in2_pin, 0)

    # Send PWM values to the motors
    pi.set_PWM_dutycycle(left_mtr_spd_pin, L_PWM_val)
    pi.set_PWM_dutycycle(right_mtr_spd_pin, R_PWM_val)

left_pos = 0
def left_enc_callback(tick):
    """Add 1 tick (either +1 or -1)"""
    
    global left_pos
    left_pos += tick

right_pos = 0
def right_enc_callback(tick):
    """Add 1 tick (either +1 or -1)"""
    
    global right_pos
    right_pos += tick

if __name__ == '__main__':

    # Set up to count encoder pulses
    pi = pigpio.pi()
    left_decoder = Decoder(pi, left_enc_A_pin, left_enc_B_pin, left_enc_callback)
    right_decoder = Decoder(pi, right_enc_A_pin, right_enc_B_pin, right_enc_callback)

    # Set up initial values
    rospy.init_node('wheels', anonymous=True)
    rate = rospy.Rate(10)
    prev_left_pos = 0
    prev_right_pos = 0
    set_mtr_spd(pi, 0, 0, L_mode, R_mode)
    speeds = range(240, 60, -20)  # list of speeds at which to drive
    data = ["Lft_TR, Rgt_TR, Avg_TR, spd\n",]  # Header

    for spd in speeds:
        # set direction
        L_mode = 'FWD'
        R_mode = 'FWD'
        prev_time = rospy.Time.now().to_sec()
        start_time = prev_time
        delta_time = 0.0
        while True:
            # Calculate actual tick rate for left & right wheels
            delta_left_pos = left_pos - prev_left_pos
            delta_right_pos = right_pos - prev_right_pos
            prev_left_pos = left_pos
            prev_right_pos = right_pos
            curr_time = rospy.Time.now().to_sec()
            delta_time = curr_time - prev_time
            prev_time = curr_time
            L_atr = delta_left_pos / delta_time
            R_atr = delta_right_pos / delta_time

            # Let speed stabilize before collecting data
            if curr_time - start_time < DWELL/8:
                L_atr_list = []
                R_atr_list = []

            # Collect data
            else:
                L_atr_list.append(L_atr)
                R_atr_list.append(R_atr)

            # Process data before changing speed
            if curr_time - start_time > DWELL:
                print(f"Done with spd={spd}")
                # find average values of L_atr and R_atr
                L_avg = sum(L_atr_list)/len(L_atr_list)
                R_avg = sum(R_atr_list)/len(R_atr_list)
                combined = (L_avg + R_avg)/2
                data.append(f"{L_avg:.2f}, {R_avg:.2f}, {combined:.2f}, {spd}\n")
                break

            # Set motor speeds
            set_mtr_spd(pi, spd, spd, L_mode, R_mode)
            

            rate.sleep()
    print("Finished LOOP")
    with open(datafile, 'w') as f:
        for line in data:
            f.write(line)
        
    # Turn off motors
    set_mtr_spd(pi, 0, 0, 'OFF', 'OFF')

    # Clean up & exit
    print("\nExiting")
    left_decoder.cancel()
    right_decoder.cancel()
    pi.set_servo_pulsewidth(left_mtr_spd_pin, 0)
    pi.set_servo_pulsewidth(right_mtr_spd_pin, 0)
    pi.stop()
