# rplidar_motor_control
A utility node to control the rplidar's motor by calling the rplidar_node services /stop_motor or /start_motor based upon whether or not certain other nodes are running. Part of the practical_robot_utils collection at https://github.com/lbrombach/practical_robot_tools.git.

Author: Lloyd Brombach <br> 
lbrombach2@gmail.com  <br> 
November 2021
<br><hr><br> 

 
## Description:
The RPLidar's motor is always running by default. This can prematurely wear the bearings and reduce robot run-time if running when not needed. This, along with some base robot management packages provided by you, allows for the automatic starting and stopping of the RPLidar motor when certain nodes are started or stopped (Default are rviz and move_base, but user-settable). The "base robot management package" may be as simple as a launch file or script that starts the rplidar_node and the rplidar_motor_control node. 

## Operation:
The rplidar_node does provide services to start and stop the motor, but manually calling the services on the command line is cumbersome. This node polls the ROS master to get a list of active nodes. If either of the two modes specified by the user are up, the node presumes that the lidar is likely to be needed. It then listens for the specified scan topic to determine if the rplidar is actively scanning and calls the motor_start and motor_stop services provided by the rplidar_node as needed. The node checks for two nodes. If you only want to monitor for one node simply set both parameters to the same node name. Keep in mind that the node will see a match if any string in the node list contains the string in the node1 or node2 parameter. This means "rviz" will see rviz running even if rviz is registered as "rviz-12kmsflkeetblahblahblah." 

## Usage:
1. Plug in your RPLidar and launch the rplidar_node how you normally would (the motor should be running). You can either watch the RPLidar turret or watch the output of rostopic echo scan. 
2. ***rosrun rplidar_motor_control rplidar_motor_control***. If the scan topic is "scan" that's all you have to do, otherwise remap the topic name or set the topic_name parameter before starting this node. 
3. Verify that the motor stops a few moments after starting - if neither of the monitored nodes are running you should see the screen output that it is calling the stop_motor service and the motor should stop.
4. Now start rviz with rosrun rviz rviz and in a moment you should see an output on the screen the turret should start.
5. Add the rplidar_node and the rplidar_motor_control to a launch file that happens whenever the robot is on. Congrats - your RPLidar motor is now automated.


## Connections:
Uses the std_srvs/Empty motor_start and std_srvs/Empty motor_stop services as provided by the [rplidar_node](http://wiki.ros.org/rplidar) or any other node with those topic names.

## Subscriptions
- scan ([sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html))
    - The laser scan message published by the RPLidar you wish to control

## Publications
none

## Parameters
- rplidar_motor_controller/topic_name (string, default: scan)
    - Sets the scan subscriber to this parameter

- rplidar_motor_controller/node1 (string, default: rviz)
    - The name of first of two nodes to check for

- rplidar_motor_controller/node2 (string, default: move_base)
    - The name of second of two nodes to check for


