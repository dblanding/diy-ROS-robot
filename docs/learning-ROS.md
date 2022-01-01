# LEARNING ROS: How I came up the ROS learning curve.
ROS has a reputaion for having a pretty formidable learning curve. So when I decided to tackle learning ROS, I knew it would be a pretty long slog. Nevertheless, I already had some of the prerequisite skills. I knew my way (pretty well) around these subjects:
* Linux
* Python
* Raspberry Pi and its gpio to interface with sensors and actuators

Initially, I didn't have ROS installed on my PC. So I got started by signing up for the [Robot Ignite Academy](https://www.theconstructsim.com) online courses:
* ROS Basics in 5 Days (Python)
* ROS Navigation in 5 days

These courses are presented through a web browser interface, with all the ROS installation and configuration happening at the server end. So the student is not distracted by needing to deal with any of this, but can focus on the topic being taught. The student can launch some very impressive demonstrations with a minimum number of keystrokes, providing very strong positive reinforcement. So that's a good thing. But after completing these 2 courses, I realized that I needed to jump into the real world and learn how to install and configure ROS on my own, so that I could build my own robot.

So, here is the summary of that process.

* I installed ROS on my Raspberry Pi
* I ran through a bunch of [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
* I did the [TurtleBot3 Simulation Tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
* I then undertook to build [my own robot](https://github.com/dblanding/diy-ROS-robot) with capabilities similar to TB3.
* Once I started building my own robot, certainly the **best** investment I made was to purchase Lloyd Brombach's excellent book [Practical Robotics in C++](https://www.amazon.com/Practical-Robotics-Program-Autonomous-Raspberry-ebook/dp/B08VDP2ZP5). It's up-to-date and packed with practical knowledge about every aspect of building your first robot. And it's very clearly written. I was afraid that the C++ would make it dry (Since I'm a Python guy), but even the C++ code was presented in a way that made it easy to grasp.

[Here](sticky-spots.md) is a summary of some of the problems I encountered along the way and how I resolved them.

