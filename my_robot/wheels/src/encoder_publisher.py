#!/usr/bin/env python3
""" ROS node for driving wheel motors and reading their encoders.

Subscribe to topic:
    /cmd_vel
    
Publish on topics:
    /right_ticks
    /left_ticks
    /act_vel

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

TICKS_PER_REV = 690
WHEEL_CIRCUMFERENCE = 0.213  # meters
TICKS_PER_METER = TICKS_PER_REV / WHEEL_CIRCUMFERENCE
TRACK_WIDTH = .187  # Wheel Separation Distance (meters)
MIN_PWM_VAL = 63  # Minimum PWM value that motors will turn
MAX_PWM_VAL = 255  # Maximum allowable PWM value
MIN_X_VEL = 0.1  # minimum x velocity robot can manage (m/s)
turning_in_place = False  # Flag signifying robot is turning in place
new_ttr = False  # Flag signifying target tick rate values are new
L_ttr = 0  # Left wheel target tick rate
R_ttr = 0  # Right wheel target tick rate
L_spd = 0  # Left motor speed (pos 8-bit int) for PWM signal
R_spd = 0  # Right motor speed (pos 8-bit int) for PWM signal
L_mode = 'OFF'  # motor mode: 'FWD', 'REV', 'OFF'
R_mode = 'OFF'

# PID stuff
KP = 0.5  # Proportional coeff
KD = 0.1  # Derivative coeff
L_prev_err = 0
R_prev_err = 0
MAX_PID_TRIM = 20  # Max allowable value for PID trim term

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

    The empirical relationship between target tick rate and the spd signal
    sent to the motors has been found to roughly follow a parabolic curve
    with shallower slope at low speed. This relationship is pretty closely
    approximated by a piecewise linear curve comprised of 3 linear segments.
    The tick_rate & mtr_spd values of the segment end points are defined
    in the tuple TRS_CURVE ((tr3, sp3), (tr2, sp2), (tr1, sp1), (tr0, sp0))

    In the present algorithm, the tick rate is examined to see which segment
    applies to it, starting with the steepest (highest value). The applicable
    slope and intercept are then used to calculate the value of spd.
    """
    global TRS_COEFF
    tick_rate = abs(tick_rate)

    # This just gets done once.
    if not TRS_COEFF:
        # Build list of coefficients (lower_limit, m, b) for each segment
        # m=slope, b=intercept, applicable for tick_rate > lower_limit
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

    # This happens every time
    spd = 0
    for lower_limit, m, b in TRS_COEFF:
        if tick_rate > lower_limit:
            spd = int(m * tick_rate + b)
            break
    return spd
    
def set_mtr_spd(pi, latr, ratr):
    """
    Derive motor speed and mode from L & R target tick rates. Drive motors.

    Target tick rates are converted to "best guess" PWM values to drive the
    motors using an empirical curve.
    
    Tick rates can be either positive or negative, wheras motor speed (spd)
    will always be a positive 8-bit int (0-255). Therefore, it is needed to
    specify a mode for the motors: FWD, REV, or OFF.

    At low speeds, a very small PWM signal struggles to get the motor to
    overcome an inherently unknowable amount of friction. This makes it very
    difficult for the robot to accurately follow the command velocity.

    To improve the robot's ability to accurately follow the command velocity,
    (especially noticeable when making slow, in-place turns) PID feedback is
    used to minimize the difference between target and actual tick rate (error).
    """

    global new_ttr, L_spd, R_spd, L_mode, R_mode, L_prev_err, R_prev_err

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

    # Find tick rate error
    L_err = abs(L_ttr) - abs(latr)
    R_err = abs(R_ttr) - abs(ratr)

    # Calculate proportional term
    L_pro = int(L_err * KP)
    R_pro = int(R_err * KP)

    # Calculate derivative term
    L_der = int((L_err - L_prev_err) * KD)
    R_der = int((R_err - R_prev_err) * KD)
    L_prev_err = L_err
    R_prev_err = R_err

    # Combine terms and apply max limit
    L_pid_trim = L_pro + L_der
    R_pid_trim = R_pro + R_der

    # Limit value of PID trim
    if L_pid_trim > MAX_PID_TRIM:
        L_pid_trim = MAX_PID_TRIM
    elif L_pid_trim < -MAX_PID_TRIM:
        L_pid_trim = -MAX_PID_TRIM
    if R_pid_trim > MAX_PID_TRIM:
        R_pid_trim = MAX_PID_TRIM
    elif R_pid_trim < -MAX_PID_TRIM:
        R_pid_trim = -MAX_PID_TRIM

    # Add PID feedback to minimize error 
    L_PWM_val = L_spd + L_pid_trim
    R_PWM_val = R_spd + R_pid_trim

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

    # Set up to publish actual velocity
    actual_vel = Twist()
    robot_vel = rospy.Publisher('act_vel', Twist, queue_size=10)

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

        # Calculate robot actual velocity
        x_vel = ((R_atr + L_atr) / 2) / TICKS_PER_METER  # meters/sec
        z_vel = ((R_atr - L_atr) / TRACK_WIDTH) / TICKS_PER_METER  # rad/sec
        prev_L_atr = L_atr
        prev_R_atr = R_atr

        # Publish robot actual velocity
        actual_vel.linear.x = x_vel
        actual_vel.angular.z = z_vel
        robot_vel.publish(actual_vel)

        # Set motor speeds
        set_mtr_spd(pi, L_atr, R_atr)

        rate.sleep()

    # Clean up & exit
    print("\nExiting")
    left_decoder.cancel()
    right_decoder.cancel()
    pi.set_servo_pulsewidth(left_mtr_spd_pin, 0)
    pi.set_servo_pulsewidth(right_mtr_spd_pin, 0)
    pi.stop()
