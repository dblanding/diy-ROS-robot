# Building the DIY Robot:
### Places where I got "stuck" along the way and how I got "unstuck"
ROS is big. It encompasses a lot of very useful and powerful software. So there is no way around the truth that there will be lots of places where you can get stuck when coming up the learning curve.
Here is my retrospective recollection of some of the places where I got stuck as I was building the DIY Robot and how I eventually resolved whatever it was that blocked my progress. I do this for the benefit of both my "future self" as well as for others who may be on a similar learning trajectory.

* **The IMU** - On [a previous robot](https://github.com/dblanding/lidar-scan-map) I used the Bosch BNO08x IMU in RVC mode which just published Euler angles and worked pretty well for that. However, on this project, I needed to publish IMU data on the ROS /imu topic, so my selection of IMU was based not so much on the IMU device itself, but more on how well the available libraries worked with ROS. This made me decide on using the older Bosch BNO055 device for which I found a very good [ROS library](https://github.com/RoboticArts/ros_imu_bno055). However, I discovered a couple of small typos/errors which I emailed the maintainer about. The first was just a tiny typo in the readme. The second had to do with setting up udev rules to make sure that each of the devices I had plugged in to the USB ports would be correctly identified by their associated nodes.
```First, while following your advice about udev rules in your readme.md, I noticed a small typo:
$ sudo cp utils/99-bno055.rules /etc/udev/rules.
should be:
$ sudo cp utils/99-bno055.rules /etc/udev/rules.d
Secondly, I discovered that you have ttyUSB0 hard coded into your driver code.
At first, with the the symlinks as shown below, my rplidar node wasn't able to publish on its /scan topic:
ubuntu@robot:~/catkin_ws/src/my_robot/bringup/launch$ ls -al /dev | grep ttyUSB
crw-rw---- 1 root dialout 188,
1 Nov 19 13:51 ttyUSB1
crw-rw---- 1 root dialout 188,
2 Nov 19 13:44 ttyUSB2
lrwxrwxrwx 1 root root
7 Sep 7 18:37 ttyUSB_IMU -> ttyUSB1
lrwxrwxrwx 1 root root
7 Nov 19 13:44 ttyUSB_LIDAR -> ttyUSB2
Then I unplugged the rplidar, rebooted and got the ttyUSB_IMU symlink pointing to ttyUSB0.
I then plugged in the rplidar and this allowed my lidar data to be published. The symlinks were now as shown below:
ubuntu@robot:~/catkin_ws/src/my_robot/rplidar_ros/launch$ ls -al /dev | grep ttyUSB
crw-rw---- 1 root dialout 188,
0 Nov 19 14:06 ttyUSB0
crw-rw---- 1 root dialout 188,
1 Nov 19 14:07 ttyUSB1
lrwxrwxrwx 1 root root
7 Sep 7 18:37 ttyUSB_IMU -> ttyUSB1
lrwxrwxrwx 1 root root
7 Nov 19 14:01 ttyUSB_LIDAR -> ttyUSB0
This made me realize that "ttyUSB0" must be hardcoded into your driver. So I changed "ttyUSB0" to "ttyUSB_IMU" on
my copy but I thought you might want to fix it differently than the way I have done it.
```
* **RPLidar A1** - No problems whatsoever. This works perfectly right out of the box. Now that I learned how to set up udev rules, I set it up so my node will be able to easily find it at /dev/ttyUSB_LIDAR
* **Sensor Fusion** using robot_pose_ekf was pretty stubborn. I used `rosrun robot_pose_ekf wtf.py` to help figure it out. Eventually, I had to do these things to get it to work:
    * In the /imu/data message publisher:
        * I had to normalize the quaternions coming from the imu publisher. ((x, y, z, w) were all integers between 0 - 16,384)
        * I had to get rid of the '-1' values in the first element of the covariance matrices.
    * In the odometry message publisher:
        * I had to seed the diagonal elements of the covariance matrix with small (non-zero) values. (0.01)
        * I had to remove the tf broadcaster broadcasting transforms between base_link and odom. (robot_pose_ekf has that covered.)
    * I also moved the odometry-publisher node and robot-pose-ekf node from my desktop computer to the robot onboard computer
    * Changed odom pub rate from 4 Hz to 10 Hz (IMU pub rate is 50 Hz)
* After I wrote my **odometry_publisher** node, I noticed that while operating under the control of the navigation stack, my robot moved smoothly when travelling in the positive X direction  on the map but moved very stutteringly in the negative X direction. I eventually discovered that I had botched the Twist part of the message being published in the /odom topic. As it turns out, the velocities are supposed to be referred to the child frame (base_link), not the odom frame.
* At one point, I encountered a problem where the **catkin_make** command failed to compile all the packages in the catkin workspace. Thanks to Lloyd Brombach for identifying the problem. The solution is explained [here](https://answers.ros.org/question/54178/how-to-build-just-one-package-using-catkin_make/). I had unwittingly followed some online advice to use the command `catkin_make --only-pkg-with-deps <target_package>` and didn't realize that I needed to follow this with the command `catkin_make -DCATKIN_WHITELIST_PACKAGES=""`.
* I have found Lloyd's observation about roboticists to be spot on!
> Robotics is for the very determined and persistent

