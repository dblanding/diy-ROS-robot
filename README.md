# Do-it-yourself differential drive ROS robot
This project started out when my wife bought me a Raspberry-Pi 4 (w/ 8GB RAM) for my birthday.
At the time, I was curious about learning ROS and I also wondered if the Raspi4
would be suitable for use as a desktop computer. Could the new Raspi handle the
rigors of running ROS, and if so, wouldn't that be great to enable me to pursue my ROS education?

The answer to both questions turned out to be YES, and so this inital curiosity has led me to tackle this project. Read on for the technical details. There is also an overview summary article [here](https://dougblanding.medium.com/my-first-medium-article-4ff3eab725ec).

A Raspberry Pi 3B+ computer is used onboard the robot, while
the Raspi4 has found use as my desktop computer. Here's an early picture of the robot before the IMU was in place.

![Early robot](images/early-robot.jpg)

And here it is fully functional, powered up and ready to go.

![The DIY robot](images/robot.jpg)

Keeping in mind that my goal was to learn ROS (in general) and to build my first autonomous ROS mobile robot (in particular), I opted for the robot to be as simple as possible, with a preference for readily available components, assembled with minimal modification in a configuration that allows really easy access. I also realized that I wasn't smart enough to know at the outset what the final configuration was going to look like so I chose a configuration that made modifications simple. I deliberately avoided the urge to be clever and try to pack things too tightly into a small space.

The most complicated part I had to make was the 1/4 x 4 x 12 inch long piece of oak that I purchased from Home Depot. Using only moderate precision, I drilled holes wherever they were needed. Everything else is readily available as an off-the-shelf componenent (see [parts list](docs/parts-list.md)). For example, the entire wheel/motor/encoder assembly including the lower blue anodized plate, the upper clear plastic plate and the brass standoffs came from an [Elegoo Tumbller](https://www.amazon.com/dp/B07QWJH77V?psc=1&ref=ppx_yo2_dt_b_product_details).

![Elegoo Tumbller Self-Balancing Robot Car](images/elegoo-tumbller.jpg)

### Robot Configuration
The design of the robot is intended to be as simple as possible, incorporating only the most essential features needed to enable it to navigate autonomously.
* Lidar: [RPLidar A1](http://wiki.ros.org/rplidar)
* IMU: [Bosch BNO055 IMU](http://wiki.ros.org/ros_imu_bno055)
* 2 wheel [differential drive](docs/differential-drive.md) configuration
    * Wheels driven by DC Gearmotors with integral encoders
    * L298N motor controller board
    * Enhanced gpio performance with [pigpio library](http://abyz.me.uk/rpi/pigpio/python.html) (no need for Arduino)
        * pi gpio pins are quick enough to count encoder pulses
        * provides PWM pin output to drive motors
    * See [how motor speed control works](docs/motor-speed-control.md)

### Configuring ROS on board the robot
The `my_robot` folder contains all the code that goes in robot's catkin_ws/src directory.

[Here](docs/setup-2nd-computer.md) are the details of the configuration and setup of the robot onboard computer.


### Using the ROS Navigation Stack
This is where ROS really shines. This is what makes ROS so powerful. There is no need to reinvent all the robotics algorithms for yourself. With ROS, all the heavy lifting has been done. Just issue a couple simple commands and you get a map. A couple of clicks in RVIZ and ROS will take your robot from point A to point B (on the map that it made) without colliding with anything along the way, using sophisticated algoithms such as:
* SLAM / Mapping
* Localization using AMCL
* Path Planning to find the best route from initial pose to goal pose.

Navigation is launched from the `robot_nav` package, residing inside the `my_robot` folder. And there really isn't very much there. Just some launch files, parameters, and maps.

To operate the robot, refer to these [operating instructions](docs/operate-robot.md).

[Here](docs/ubuntu-install.md) are the details of the setup and configuration of the raspi4 computer.

### DIY robot makes its maiden voyage
On 11/27/21, the DIY robot made its first trip under the control of the ROS Navigation Stack. It wasn't a very smooth or efficient looking route (and the navigation stack was producing lots of warnings and errors) but the robot managed to complete its first trip to a goal pose specified in RVIZ.

https://user-images.githubusercontent.com/53412304/148826107-439ce96d-d5c6-48b3-8599-9f98fdcbd351.mp4

Being a novice, it wasn't obvious to me what was causing all these warnings and errors. So I decided on a "divide and conquer" approach. I decided to make my robot behave similarly to the Turtlebot3 Burger. That way I could just copy the navigation parameters from the Burger and hopefully the navigation performance would be fine. And if not, then maybe the problem is elsewhere.

Compared to the Burger, my robot was going kind of fast, so I switched to motors with a higher gear reduction. I estimated that I would need 100 RPM motors to get the robot speed down around 0.25 m/s. Unfortunately, the gearbox configuration on the motors I found was different than the current motors, so I had to make new motor brackets. Here is the robot in the new configuration.

![latest robot](images/latest-robot.jpg)

As it turns out, this was a good approach.
* Even after studying the [DWA local planner documentation](http://wiki.ros.org/dwa_local_planner), it's actually pretty frustrating to get all the parameters in the DWA_planner optimized, so being able to just copy the parameters from Burger was very helpful. I could be confident that any remaining problems with the robot's behavior while operating under the control of the navigation stack were most likely being caused by something in my code.
* This led me to discover that the Twist part of the message being published in the /odom topic was referring to the wrong frame. Here I present a [summary of this and other issues](docs/sticky-spots.md) I encountered along the way and how they were resolved.

### 12/30/2021 DIY Robot Project complete.

* Initial goal of [learning ROS](docs/learning-ROS.md) achieved.
* Secondary goal of building a robot capable of autonomous navigation also achieved.
    * Navigation parameters set to produce optimum driving performance under navigation stack control.
    * Robot now navigates smoothly and efficiently to goal pose specified in RVIZ.
    * All issues causing warnings and errors resolved.
* Here is a split screen video showing the robot making a trip from the office, through the dining room and into the kitchen in real time. Now how cool is that? Robotics is such a great hobby, thanks in large part to ROS!

https://user-images.githubusercontent.com/53412304/148745637-946173ff-d5c7-4747-9fac-c8e4dd46d131.mp4

This next video is the return trip, created on Kapwing, a free online video editing service with the capabiltiy of compositing multiple videos.

https://user-images.githubusercontent.com/53412304/150647122-91798707-5ee4-4138-861e-a0de373af5b5.mp4
