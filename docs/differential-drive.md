# Basic kinematics of differential drive vehicles
The differential drive configuration consists of exactly 2 identical but independently controlled drive wheels mounted coaxially and spaced apart from each other by a distance known as the track width, **W**. Each of the wheels, when driven in the forward direction, will impart a velocity (vel_L or vel_R respectively) at the point of contact with the floor. In the figure below, the vehicle is shown with its front pointed to the right. It has exactly two independent degrees of freedom:
1. Translation (It can drive straight ahead or straight back, perpendicular to the axis of the wheels)
2. Rotation (It can rotate about the Z axis located mid-way between the two wheels)

![Vehicle Configuration](images/my-diff-drive.png)

Our vehicle will receive messages on the **cmd_vel** topic specifying the velocity at which it is to drive in both of these two degrees of freedom. These velocities are:
* **vel_T** (meters/second)
* **vel_Theta** (radians/second)

### Convert the commanded vehicle velocities to velocities vel_L and vel_R at the wheels

To me, the most intuitive way to think about this problem is to divide it into 2 separate and independent cases, then superimpose the results of the separate cases. We can use superposition because the vehicle's 2 degrees of freedom are **independent**.

First, consider the case of pure translation with zero rotation:
* In the case of pure translation (**vel_Theta** = 0)

    * **vel_R = vel_T**
    * **vel_L = vel_T**

Next consider the case where the vehicle is turning in place. (The value of vel_T equals zero.) Hopefully, your intuition will tell you that if vel_Theta is positive, then  vel_R is also positive and they are related by the moment arm W/2. It should also be clear that vel_L must be of equal magnitude but in the opposite direction to vel_R.

* In the case of turning in place (**vel_T** = 0):
    * **vel_R = (vel_Theta * W/2)**
    * **vel_L = - (vel_Theta * W/2)**

Now we can use the principle of superposition to combine these two cases to come up with equations that are good for the general case of combined translation and rotation.

**vel_R = vel_T + (vel_Theta * W/2)**

**vel_L = vel_T - (vel_Theta * W/2)**

### Alternatively, if we know vel_L and vel_R, we can calculate the resulting vehicle velocities.
Imagine this metaphor. A cart is being drawn by 2 horses side by side, spaced apart by a yoke of length W. The cart is attached to the center of the yoke. Hopefully, it is clear that the cart's speed will be equal to the average of the speeds of the left and right horses. Applying this metaphor to our vehicle gives the following equation:

**vel_T = (vel_R + vel_L) / 2**

To find vel_Theta, imagine a slightly different metaphor where the horses are being used to rotate a "turnstile" of diameter W in the CCW direction. In this situation, the horses are now facing in opposite directions. Applying this metaphor to our vehicle gives the following equation:

**vel_Theta * W/2 = (vel_R - vel_L) / 2**

Which simplifies to:

**vel_Theta = (vel_R - vel_L) / W**

### Finding motor speed and tick rates from vel_L and vel_R
Remember that **vel_L** and **vel_R** represent speed (m/sec) at the tire surface. It is a simple matter to relate **vel_L** and **vel_R** to motor speed (RPM) and encoder tick_rate (ticks/sec) using the parameters of
* wheel diameter
* gear ratio
* encoder ticks/rev



