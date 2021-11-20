#!/usr/bin/env python3
""" ROS node for driving wheel motors and reading their encoders.

Subscribe to topic:
    /cmd_vel
    
Publish on topics:
    /right_ticks
    /left_ticks

Code adapted from rotary_encoder example at:
https://abyz.me.uk/rpi/pigpio/examples.html
Hook up A and B so that values increase when moving forward.
pigpio daemon must be running. Launch with 'sudo pigpiod'.
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32
import pigpio

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
# (approximate) linear relationship between cmd_vel_x and motor
# spd value determined empirically:  spd = M * cmd_vel_x + B
M = 80
B = 40

TICKS_PER_METER = 1880  # Number of encoder ticks per meter of travel
TRACK_WIDTH = .163  # Wheel Separation Distance (meters)
TFF = 4  # Turning Fudge Factor amplifies turning response
MIN_PWM_VAL = 30  # Minimum PWM value that motors will turn
MAX_PWM_VAL = 255  # Maximum allowable PWM value

cmd_vel_x = 0  # linear (x) component of cmd_vel (meters/sec)
cmd_vel_z = 0  # angular (theta-z) component of cmd_vel (radians/sec)

def listener_callback(msg):
    """Just save the command velocities linear.x and angular.z"""
    global cmd_vel_x, cmd_vel_z
    # Linear Components: msg.linear.x, msg.linear.y, msg.linear.z
    cmd_vel_x = msg.linear.x
    # Angular Components: msg.angular.x, msg.angular.y, msg.angular.z
    cmd_vel_z = msg.angular.z

def listener():
    rospy.Subscriber("/cmd_vel", Twist, listener_callback)

def set_mtr_spd(pi):
    """
    Command motor direction & speed from linear and angular velocities.
    """

    # Superimpose linear and angular components of cmd_vel
    vel_L_wheel = cmd_vel_x - (cmd_vel_z * (TRACK_WIDTH * TFF / 2))
    vel_R_wheel = cmd_vel_x + (cmd_vel_z * (TRACK_WIDTH * TFF / 2))

    # Set motor direction pins appropriately
    if vel_L_wheel > 0:  # Forward
        pi.write(left_mtr_in1_pin, 0)
        pi.write(left_mtr_in2_pin, 1)
    elif vel_L_wheel < 0:  # Reverse
        pi.write(left_mtr_in1_pin, 1)
        pi.write(left_mtr_in2_pin, 0)
    else:  # Parked
        pi.write(left_mtr_in1_pin, 0)
        pi.write(left_mtr_in2_pin, 0)

    if vel_R_wheel > 0:  # Forward
        pi.write(right_mtr_in1_pin, 0)
        pi.write(right_mtr_in2_pin, 1)
    elif vel_R_wheel < 0:  # Reverse
        pi.write(right_mtr_in1_pin, 1)
        pi.write(right_mtr_in2_pin, 0)
    else:  # Parked
        pi.write(right_mtr_in1_pin, 0)
        pi.write(right_mtr_in2_pin, 0)

    # Convert from velocity at wheel to motor speed PWM value
    left_spd = int(abs(vel_L_wheel) * M + B)
    if left_spd > MAX_PWM_VAL:
        left_spd = MAX_PWM_VAL
    if left_spd < MIN_PWM_VAL:
        left_spd = 0
    right_spd = int(abs(vel_R_wheel) * M + B)
    if right_spd > MAX_PWM_VAL:
        right_spd = MAX_PWM_VAL
    if right_spd < MIN_PWM_VAL:
        righ_spd = 0

    # Send speed commands to the motors
    pi.set_PWM_dutycycle(left_mtr_spd_pin, left_spd)
    pi.set_PWM_dutycycle(right_mtr_spd_pin, right_spd)


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
    rate = rospy.Rate(10) # 10hz

    # Publish encoder data
    prev_left_pos = 0
    prev_right_pos = 0
    while not rospy.is_shutdown():
        left_pub_ticks.publish(left_pos)
        right_pub_ticks.publish(right_pos)
        delta_left_pos = left_pos - prev_left_pos
        delta_right_pos = right_pos - prev_right_pos
        
        # Set motor speeds
        set_mtr_spd(pi)
        rate.sleep()

    # Clean up & exit
    print("\nExiting")
    left_decoder.cancel()
    right_decoder.cancel()
    pi.set_servo_pulsewidth(left_mtr_spd_pin, 0)
    pi.set_servo_pulsewidth(right_mtr_spd_pin, 0)
    pi.stop()
