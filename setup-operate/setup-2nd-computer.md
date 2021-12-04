# Setup Raspberry Pi 3B+ as robot onboard computer

* Install Ubuntu-server 20.04 LTS
* Set name to 'robot'
* Install ros-base
* Go through tutorial [Running ROS across multiple machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
    * Install ros-tutorials on robot: `$ sudo apt-get install ros-noetic-ros-tutorials`
    * Add each other's host name in /etc/hosts file
* Edit ~/.bashrc to source ~/catkin_ws/devel/setup.bash
* Add [rplidar package](rplidar/rplidar.md)

* Set up passwordless ssh login to robot, following the steps at [How to setup SSH login without password on Linux systems](https://thelinuxgurus.com/how-to-setup-ssh-login-without-password/).

* Install rosserial `sudo apt install ros-noetic-rosserial`

* [Download & Install pigpio](http://abyz.me.uk/rpi/pigpio/download.html)
* Install python mathutils library `pip install mathutils` (used to normalize quaternion)
* Install robot_pose_ekf `sudo apt install ros-noetic-robot-pose-ekf`
* Install gmapping and navigation
    * `sudo apt install ros-noetic-gmapping`
    * `sudo apt install ros-noetic-navigation`
* Install udev rules to correctly identify USB ports on which IMU and Laser are connected
    * [ROS driver for the BNO055 IMU using serial communication](https://github.com/RoboticArts/ros_imu_bno055) shows how to do it for the BNO055 imu.
```
$ roscd ros_imu_bno055
$ sudo cp utils/99-bno055.rules /etc/udev/rules.d
$ sudo udevadm control --reload-rules && sudo udevadm trigger
```
Here is file `99-bno055.rules`
```
# Udev rule for Serial to USB converter with FTID FT232R chip for communication 
# with IMU BNO055
#
# Check dev info: udevadm info -a -p  $(udevadm info -q path -n /dev/ttyUSB0)
# Reload udev: sudo udevadm control --reload-rules & udevadm trigger
#

ACTION=="add", ATTRS{product}=="FT232R USB UART", ATTRS{manufacturer}=="FTDI", SYMLINK+="ttyUSB_IMU"
```
* Then do the same with the RPLidar. Here is file `99-rplidar.rules`
```
# Udev rule for Serial to USB converter for communication with RPLidar
#
# Check dev info: udevadm info -a -p  $(udevadm info -q path -n /dev/ttyUSB1)
# Reload udev: sudo udevadm control --reload-rules & udevadm trigger
#

ACTION=="add", ATTRS{product}=="CP2102 USB to UART Bridge Controller", ATTRS{manufacturer}=="Silicon Labs", SYMLINK+="ttyUSB_LIDAR"
```
After rebooting, both symlinks were in place, but I wasn't able to see any laser scan data.
```
ubuntu@robot:~/catkin_ws/src/my_robot/bringup/launch$ ls -al /dev | grep ttyUSB
crw-rw----  1 root dialout 188,   1 Nov 19 13:51 ttyUSB1
crw-rw----  1 root dialout 188,   2 Nov 19 13:44 ttyUSB2
lrwxrwxrwx  1 root root           7 Sep  7 18:37 ttyUSB_IMU -> ttyUSB1
lrwxrwxrwx  1 root root           7 Nov 19 13:44 ttyUSB_LIDAR -> ttyUSB2
```
Subsequently, I rebooted robot with the rplidar unplugged at first.
I then plugged it in and got ttyUSB_LIDAR -> ttyUSB0.
This time the laser data was visible in RVIZ.
```
ubuntu@robot:~/catkin_ws/src/my_robot/rplidar_ros/launch$ ls -al /dev | grep ttyUSB
crw-rw----  1 root dialout 188,   0 Nov 19 14:06 ttyUSB0
crw-rw----  1 root dialout 188,   1 Nov 19 14:07 ttyUSB1
lrwxrwxrwx  1 root root           7 Sep  7 18:37 ttyUSB_IMU -> ttyUSB1
lrwxrwxrwx  1 root root           7 Nov 19 14:01 ttyUSB_LIDAR -> ttyUSB0
```

