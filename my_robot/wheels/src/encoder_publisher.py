#!/usr/bin/env python3
""" ROS node for driving wheel motors and reading their encoders.

Subscribe to topic:
    /cmd_vel
    
Publish on topics:
    /right_ticks
    /left_ticks

Decoder class adapted from rotary_encoder example at:
https://abyz.me.uk/rpi/pigpio/examples.html
Hook up A and B so that values increase when moving forward.
pigpio daemon must be running. Launch with 'sudo pigpiod'.
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32
import pigpio
import time

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

# End points of line segments in descending order ((tr, spd), ...)
TRS_CURVE = ((621, 145), (439, 109), (273, 92), (121, 83))
TRS_COEFF = None
DEBUG = True
TICKS_PER_REV = 690
WHEEL_CIRCUMFERENCE = 0.213  # meters
TICKS_PER_METER = TICKS_PER_REV / WHEEL_CIRCUMFERENCE
TRACK_WIDTH = .187  # Wheel Separation Distance (meters)
MIN_PWM_VAL = 63  # Minimum PWM value that motors will turn
MAX_PWM_VAL = 255  # Maximum allowable PWM value
MTR_TRIM = 0  # int value to trim R/L motor diff (+ to boost L mtr)
MIN_X_VEL = 0.1  # minimum x velocity robot can manage (m/s)
TURNING_FACTOR = 0.6  # compensation needed when making in-place turns
CYCLE_BOOST_THRESHOLD = 60  # spd threshold for cycle-boosting in-place turns
CYCLE_BOOST = 1.0  # cycle-boost factor to be applied on alternate cycles
turning_in_place = False  # Flag signifying robot is turning in place
new_ttr = False  # Flag signifying target tick rate values are new
L_ttr = 0  # Left wheel target tick rate
R_ttr = 0  # Right wheel target tick rate
L_spd = 0  # Left motor speed (pos 8-bit int) for PWM signal
R_spd = 0  # Right motor speed (pos 8-bit int) for PWM signal
L_mode = 'OFF'  # motor mode: 'FWD', 'REV', 'OFF'
R_mode = 'OFF'

def listener():
    """Listen to cmd_vel topic. Send Twist msg to callback. """
    rospy.Subscriber("/cmd_vel", Twist, listener_callback)

def listener_callback(msg):
    """Extract linear.x and angular.z from Twist msg."""
    cmd_vel_x = msg.linear.x  # meters/sec
    cmd_vel_z = msg.angular.z  # radians/sec
    convert_cmd_vels_to_target_tick_rates(cmd_vel_x, cmd_vel_z)

def convert_cmd_vels_to_target_tick_rates(x, theta):
    """
    Convert x and theta (target velocities) to L and R target tick rates.

    Save target tick rates in global values.
    Set global flag to signify whether target tick rate values are new.
    Set global flag to signify whether robot is turning in place.
    """

    global L_ttr, R_ttr, new_ttr, turning_in_place

    # Set global flag to signify turning in place.
    if x < MIN_X_VEL and theta:
        turning_in_place = True
    else:
        turning_in_place = False
    
    # Superimpose linear and angular components of robot velocity
    vel_L_wheel = x - (theta * (TRACK_WIDTH / 2))
    vel_R_wheel = x + (theta * (TRACK_WIDTH / 2))

    # Convert wheel velocity to tick rate
    L_rate = vel_L_wheel * TICKS_PER_METER
    R_rate = vel_R_wheel * TICKS_PER_METER

    # Are these new values?
    if L_ttr != L_rate:
        L_ttr = L_rate
        new_ttr = True
    if R_ttr != R_rate:
        R_ttr = R_rate
        new_ttr = True

def tr_to_spd(tick_rate):
    """Convert tick_rate to spd (positive integer). Return spd.

    The relation between tick_rate and mtr_spd has been found empirically to be
    pretty well approximated by a piecewise linear curve comprised of 3 linear
    segments. The tick_rate & mtr_spd values of the segment end points are defined
    in the tuple TRS_CURVE ((tr3, sp3), (tr2, sp2), (tr1, sp1), (tr0, sp0))

    In the present algorithm, the tick rate is examined to see which segment
    applies to it, starting with the steepest (highest value). The applicable
    slope and intercept are then used to calculate the value of spd.
    """
    global TRS_COEFF
    tick_rate = abs(tick_rate)
    if not TRS_COEFF:
        # This just gets done once. Build a list of coefficients
        # (tr_lower_limit, m, b) for each segment
        # m=slope, b=intercept, applicable for tick_rate > tr_lower_limit
        TRS_COEFF = []
        tr2 = 0
        sp2 = 0
        for tr1, sp1 in TRS_CURVE:
            if tr2:
                m = (sp2 - sp1) / (tr2 - tr1)
                b = sp1 - m * tr1
                coeffs = (tr1, m, b)
                TRS_COEFF.append(coeffs)
            tr2 = tr1
            sp2 = sp1
        rospy.loginfo(TRS_COEFF)
    spd = 0
    for lower_limit, m, b in TRS_COEFF:
        if tick_rate > lower_limit:
            spd = int(m * tick_rate + b)
            break
    return spd
    
def set_mtr_spd(pi, latr, ratr, newness=0, cycle=0):
    """
    Derive motor speed and mode from L & R target tick rates. Drive motors.

    Take care when converting from tick rates to motor speed. Tick rates can
    be either positive or negative, wheras motor speed (spd) will always be
    a non-negative 8-bit int (0-255). Therefore, we will also need to specify
    a mode for the motors: FWD, REV, or OFF. One reason for the "OFF" mode
    owes to the static friction of the motors. If we command a motor speed
    too low, the motors will just whine and not move.

    The empirical relationship between target tick rate and the spd signal
    sent to the motors has been found to roughly follow a parabolic curve with
    shallower slope at low speed. This relationship is encapsulated in the
    helper function tr_to_spd().

    The problem is at the low end, when a very small PWM signal struggles to
    get the motor to overcome an inherently unknowable amount of friction.
    The problem is especially noticeable when making slow, in-place turns.
    There are a few techniques that can be used to mitigate this problem. 

    The caller of this function can supply an integer value as the newness
    parameter. This value is added to the PWM signal sent to the motors, thus
    giving a temporary initial boost to get things rolling. The caller then
    decrements the newness value toward zero in subsequent calls.

    Another technique to enhance low-speed response is to boost the PWM signal
    by CYCLE_BOOST on every other cycle for in-place turns at speeds less than
    CYCLE_BOOST_THRESHOLD.

    Yet another technique would be to use a PID feedback loop to minimize the
    difference (error) between target and actual tick rate.
    """

    global new_ttr, L_spd, R_spd, L_mode, R_mode

    # Calculate new spd values when target tick rate changes
    if new_ttr:
        L_spd = tr_to_spd(L_ttr)
        R_spd = tr_to_spd(R_ttr)

        # Determine L & R modes
        if L_ttr > 0:
            L_mode = 'FWD'
        elif L_ttr < 0:
            L_mode = 'REV'
        else:
            L_mode = 'OFF'

        if R_ttr > 0:
            R_mode = 'FWD'
        elif R_ttr < 0:
            R_mode = 'REV'
        else:
            R_mode = 'OFF'

        # Reset flag
        new_ttr = False
        
        if DEBUG:
            rospy.loginfo(f"mtr spd L: {L_spd}\tR: {R_spd}")
            rospy.loginfo(f"target tr L: {L_ttr}\tR: {R_ttr}")

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

    # Driving FWD or REV? Apply MTR_TRIM and newness compensation
    if (L_mode == 'FWD' and R_mode == 'FWD') or (L_mode == 'REV' and R_mode == 'REV'):
        L_PWM_val = L_spd + MTR_TRIM + newness
        R_PWM_val = R_spd - MTR_TRIM + newness

    # Turning in place?
    else:
        L_PWM_val = L_spd + newness
        R_PWM_val = R_spd + newness

    # Apply boost compensation to low values on every other cycle
    if cycle % 2:
        if L_PWM_val < CYCLE_BOOST_THRESHOLD:
            L_PWM_val *= CYCLE_BOOST
        if R_PWM_val < CYCLE_BOOST_THRESHOLD:
            R_PWM_val *= CYCLE_BOOST

    # Limit PWM values to acceptable range
    if R_PWM_val > MAX_PWM_VAL:
        R_PWM_val = MAX_PWM_VAL
    elif R_PWM_val < MIN_PWM_VAL:
        R_PWM_val = 0

    if L_PWM_val > MAX_PWM_VAL:
        L_PWM_val = MAX_PWM_VAL
    elif L_PWM_val < MIN_PWM_VAL:
        L_PWM_val = 0

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


class Decoder:
   """Class to decode mechanical rotary encoder pulses. """

   def __init__(self, pi, gpioA, gpioB, callback):
      """
      Instantiate the class with the pi and gpios connected to
      rotary encoder contacts A and B. The callback is
      called when the rotary encoder is turned.  It takes
      one parameter which is +1 for CW and -1 for CCW.
      """

      self.pi = pi
      self.gpioA = gpioA
      self.gpioB = gpioB
      self.callback = callback

      self.levA = 0
      self.levB = 0

      self.lastGpio = None

      self.pi.set_mode(gpioA, pigpio.INPUT)
      self.pi.set_mode(gpioB, pigpio.INPUT)

      self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
      self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

      self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
      self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

   def _pulse(self, gpio, level, tick):
      """
      Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

              +---------+         +---------+           0
              |         |         |         |
         B    |         |         |         |
              |         |         |         |
         -----+         +---------+         +---------+ 1
      """

      if gpio == self.gpioA:
         self.levA = level
      else:
         self.levB = level;

      if gpio != self.lastGpio: # debounce
         self.lastGpio = gpio

         if   gpio == self.gpioA and level == 1:
            if self.levB == 1:
               self.callback(1)
         elif gpio == self.gpioB and level == 1:
            if self.levA == 1:
               self.callback(-1)

   def cancel(self):
      """
      Cancel the rotary encoder decoder.
      """

      self.cbA.cancel()
      self.cbB.cancel()


if __name__ == '__main__':

    # Listen to /cmd_vel topic
    listener()

    # Set up to publish encoder data
    pi = pigpio.pi()
    left_decoder = Decoder(pi, left_enc_A_pin, left_enc_B_pin, left_enc_callback)
    left_pub_ticks = rospy.Publisher('left_ticks', Int32, queue_size=10)
    right_decoder = Decoder(pi, right_enc_A_pin, right_enc_B_pin, right_enc_callback)
    right_pub_ticks = rospy.Publisher('right_ticks', Int32, queue_size=10)
    rospy.init_node('wheels', anonymous=True)
    rate = rospy.Rate(10)

    # Publish encoder data
    prev_left_pos = 0
    prev_right_pos = 0
    prev_L_atr = 0
    prev_R_atr = 0
    newness = 0
    prev_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        # publish cumulative tick data
        left_pub_ticks.publish(left_pos)
        right_pub_ticks.publish(right_pos)

        # Calculate actual tick rate for left & right wheels
        delta_left_pos = left_pos - prev_left_pos
        delta_right_pos = right_pos - prev_right_pos
        prev_left_pos = left_pos
        prev_right_pos = right_pos
        curr_time = rospy.Time.now().to_sec()
        delta_time = curr_time - prev_time
        prev_time = rospy.Time.now().to_sec()
        L_atr = delta_left_pos / delta_time
        R_atr = delta_right_pos / delta_time

        # Show the robot's actual velocities: x and z (theta)
        if DEBUG:
            if L_atr != prev_L_atr or R_atr != prev_R_atr:
                x_vel = ((R_atr + L_atr) / 2) / TICKS_PER_METER  # meters/sec
                z_vel = ((R_atr - L_atr) / TRACK_WIDTH) / TICKS_PER_METER  # rad/sec
                rospy.loginfo(f"X Vel = {x_vel:.2f} m/s\tZ Vel = {z_vel:.2f} rad/s")
                prev_L_atr = L_atr
                prev_R_atr = R_atr

        # Initiating a turn in place?
        if new_ttr and turning_in_place:
            newness = 12  # 0 to disable
            cycle_counter = 1
        elif turning_in_place:
            cycle_counter += 1
        else:
            cycle_counter = 0

        # Set motor speeds
        set_mtr_spd(pi, L_atr, R_atr, newness, cycle=cycle_counter)

        # Decrement newness toward zero
        if newness:
            newness -= 1

        rate.sleep()

    # Clean up & exit
    print("\nExiting")
    left_decoder.cancel()
    right_decoder.cancel()
    pi.set_servo_pulsewidth(left_mtr_spd_pin, 0)
    pi.set_servo_pulsewidth(right_mtr_spd_pin, 0)
    pi.stop()
