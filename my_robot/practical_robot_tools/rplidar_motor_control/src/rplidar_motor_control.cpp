/*********************************************************************
*rplidar_motor_control.cpp is a utility to control the rplidar's motor
*by calling the rplidar_node services /stop_motor or /start_motor
*based upon whether or not certain other nodes are running.
*Part of the practical_robot_utils collection at
*https://github.com/lbrombach/practical_robot_tools.git
*
*Author: Lloyd Brombach
*lbrombach2@gmail.com
*November 2021
**********************************************************************/
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <string>

using namespace ros;

const int SECONDS_BETWEEN_TRIES = 5; 	//too fast can cause service to hang and block forever
double lastScan = -1; 					//store time of last scan recieved

//scan callback - just store time of most recent scan received
void updateRPLidarScan(const sensor_msgs::LaserScan& scan){
	lastScan = ros::Time::now().toSec();
}

//has a scan been recieved in last one second?
bool IsScanning(){
	return ros::Time::now().toSec() - lastScan < 1;
}

//helper to calculate time elapsed since a given time
int getSecondsSince(double lastTime){
	return ros::Time::now().toSec() - lastTime;
}

//helper to check if string is found in a list of nodes
bool isPresent(std::vector<std::string> &nodes, std::string str){
	for (int i =0; i < nodes.size(); i++){
		if(nodes[i].find(str) != std::string::npos){
			return true;
		}
	}
	return false;
}

	
int main(int argc, char **argv)
{
	//normal ROS node setup: Register node with master,  advertise publisher
	ros::init(argc, argv, "rplidar_motor_control");
	ros::NodeHandle node;
	ros::ServiceClient stopLidarMotor = node.serviceClient<std_srvs::Empty>("/stop_motor");
	ros::ServiceClient startLidarMotor = node.serviceClient<std_srvs::Empty>("/start_motor");
	
	std::string topic_name;
	ros::param::param<std::string>("/rplidar_motor_controller/topic_name", topic_name, "/scan");
	ROS_INFO("Starting rplidar_motor_control node with scan topic: %s", topic_name.c_str()); 
	
	ros::Subscriber subRPLidar = node.subscribe(topic_name, 1, updateRPLidarScan);
	
	std::string node1;;
	ros::param::param<std::string>("/rplidar_motor_controller/node1", node1, "rviz");
	ROS_INFO("Starting rplidar_motor_control node with node1 name: %s", node1.c_str());

	std::string node2;
	ros::param::param<std::string>("/rplidar_motor_controller/node2", node2, "move_base"); 
	ROS_INFO("Starting rplidar_motor_control node with node2 name: %s", node2.c_str());

	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		ros::spinOnce();
		static double lastCall = ros::Time::now().toSec();
		
		//get list of running nodes from ros master
		std::vector<std::string> nodes;
		ros::master::getNodes(nodes);
		
		//check if certain nodes are running
		bool lidarRequired = isPresent(nodes, node1) || isPresent(nodes, node2) ;
		
		
		//start lidar motor if needed and not already scanning
		if(lidarRequired && !IsScanning() 
			&& getSecondsSince(lastCall) > SECONDS_BETWEEN_TRIES
			&& ros::service::exists("start_motor", true)){
				ROS_INFO("Calling motor_start");
				std_srvs::Empty msg;
				startLidarMotor.call(msg);
				lastCall = ros::Time::now().toSec();			
		} //turn lidar motor off if scanning connected and doesn't need to be
		else if(!lidarRequired && IsScanning() 
				&& getSecondsSince(lastCall) > SECONDS_BETWEEN_TRIES
				&& ros::service::exists("stop_motor", true)){
				ROS_INFO("Calling motor_stop");
				std_srvs::Empty msg;
				stopLidarMotor.call(msg);		
				lastCall = ros::Time::now().toSec();
		}
		loop_rate.sleep();
	}

	return 0;
}	
