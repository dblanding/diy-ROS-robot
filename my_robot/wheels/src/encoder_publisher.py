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

# An 8 bit integer value (0 - 255) is used to command motor speed
# Approximate relationship between tick_rate (x) and motor spd
# determined empirically:  spd = A * x^2 + B * x + C
# Valid at linear speeds up to around 0.25 m/sec
A = 2.24e-4
B = 0.0
C = 63.1

TICKS_PER_REV = 690
WHEEL_CIRCUMFERENCE = 0.213  # meters
TICKS_PER_METER = TICKS_PER_REV / WHEEL_CIRCUMFERENCE
TRACK_WIDTH = .187  # Wheel Separation Distance (meters)
MIN_PWM_VAL = int(C)  # Minimum PWM value that motors will turn
MAX_PWM_VAL = 255  # Maximum allowable PWM value
MTR_TRIM = 14  # int value to trim R/L motor diff (+ to boost L mtr)
MIN_X_VEL = 0.1  # minimum x velocity robot can manage (m/s)
turning_in_place = False  # Flag signifying robot is turning in place
new_ttr = False  # Flag signifying target tick rate values are new
L_ttr = 0  # Left wheel target tick rate
R_ttr = 0  # Right wheel target tick rate
L_spd = 0  # Left motor speed (pos 8-bit int) for PWM signal
R_spd = 0  # Right motor speed (pos 8-bit int) for PWM signal
L_mode = 'OFF'  # motor modes: 'FWD', 'REV', 'OFF'
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
    if x < MIN_X_VEL * TICKS_PER_METER and theta:
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

def set_mtr_spd(pi, latr, ratr, newness=0):
    """
    Derive motor speed and mode from L & R tick rates. Drive motors.

    Take care when converting from tick rates to motor speed. Tick rates can
    be either positive or negative, wheras motor speed (spd) will always be
    a non-negative 8-bit int (0-255). Therefore, we will also need to specify
    a mode for the motors: FWD, REV, or OFF. One reason for the "OFF" mode
    owes to the static friction of the motors. If we command a motor speed
    too low, the motors will just whine and not move.

    The problem of commanding the motors to make subtle, small adjustments is
    especially problematic when making a pure turn without any simultaneous
    translation in x.

    To address this problem, the caller of this function can supply an integer
    value to the newness parameter. This value is used to temporarily boost
    the PWM signal sent to the motors, as is needed during "in-place" turns.
    This is intended to give a temporary initial boost to the PWM signal to
    get things rolling. The caller should then gradually decrement the value
    of newness in subsequent calls.
    """

    global new_ttr, L_spd, R_spd, L_mode, R_mode

    # Calculate new estimated spd values when target tick rate changes
    if new_ttr:
        L_spd = int(A * pow(L_ttr, 2) + B * abs(L_ttr) + C)
        if L_ttr > 0:
            L_mode = 'FWD'
        elif L_ttr < 0:
            L_mode = 'REV'
        else:
            L_mode = 'OFF'

        R_spd = int(A * pow(R_ttr, 2) + B * abs(R_ttr) + C)
        if R_ttr > 0:
            R_mode = 'FWD'
        elif R_ttr < 0:
            R_mode = 'REV'
        else:
            R_mode = 'OFF'

        new_ttr = False
        # rospy.loginfo(f"mtr spd = {(R_spd + L_spd)/2}")

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

    # Apply R/L MTR_TRIM compensation and send PWM values to the motors
    if (L_mode == 'FWD' and R_mode == 'FWD') or (L_mode == 'REV' and R_mode == 'REV'):
        L_PWM_val = L_spd + MTR_TRIM + newness
        if L_PWM_val > MAX_PWM_VAL:
            L_PWM_val = MAX_PWM_VAL
        elif L_PWM_val < MIN_PWM_VAL:
            L_PWM_val = 0
        pi.set_PWM_dutycycle(left_mtr_spd_pin, L_PWM_val)

        R_PWM_val = R_spd - MTR_TRIM + newness
        if R_PWM_val > MAX_PWM_VAL:
            R_PWM_val = MAX_PWM_VAL
        elif R_PWM_val < MIN_PWM_VAL:
            R_PWM_val = 0
        pi.set_PWM_dutycycle(right_mtr_spd_pin, R_PWM_val)
    else:
        L_PWM_val = L_spd + newness
        if L_PWM_val > MAX_PWM_VAL:
            L_PWM_val = MAX_PWM_VAL
        elif L_PWM_val < MIN_PWM_VAL:
            L_PWM_val = 0
        pi.set_PWM_dutycycle(left_mtr_spd_pin, L_PWM_val)

        R_PWM_val = R_spd + newness
        if R_PWM_val > MAX_PWM_VAL:
            R_PWM_val = MAX_PWM_VAL
        elif R_PWM_val < MIN_PWM_VAL:
            R_PWM_val = 0
        pi.set_PWM_dutycycle(right_mtr_spd_pin, R_PWM_val)

    # Uncomment these next lines to see the robot's actual driving speed
    '''
    target_speed = ((R_ttr + L_ttr)/2) / TICKS_PER_METER
    actual_speed = ((R_atr + L_atr) / 2) / TICKS_PER_METER
    rospy.loginfo(f"Target = {target_speed:.2f}\tActual = {actual_speed:.2f} m/s")
    '''

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

        # About to initiate a turn in place?
        if new_ttr and turning_in_place:
            newness = 4

        # Set motor speeds
        set_mtr_spd(pi, L_atr, R_atr, newness)

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
