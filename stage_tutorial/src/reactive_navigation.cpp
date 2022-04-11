#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

double obstacle_distance;
bool robot_stopped;
double* readings;
bool avoidObstacleFrente;
bool avoidObstacle;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

	readings = (double*)malloc(4*sizeof(double));
	if(!robot_stopped){
		
		
		ROS_INFO("Tempo entre scans : %f", (float) msg->scan_time);
		ROS_INFO("Received a LaserScan with %i samples", (int) msg->ranges.size());
		obstacle_distance = *std::min_element (msg->ranges.begin(),msg->ranges.end());
		ROS_INFO("minimum distance to obstacle: %f", obstacle_distance);
		ROS_INFO("Distancia a frente do obstaculo: %f", msg->ranges[540]); 
		
	}
	
	readings[0] = msg->ranges[30];
	readings[1] = msg->ranges[120];
	readings[2] = msg->ranges[210];
	
	ROS_INFO("Leitura 270: %f", readings[0]);
	ROS_INFO("Leitura 540: %f", readings[1]);
	ROS_INFO("Leitura 810: %f", readings[2]);
	// ROS_INFO("Leitura 1080: %f", readings[3]);


}

geometry_msgs::Twist calculateCommand(double* & readings){
	auto cmd_vel_msg = geometry_msgs::Twist();

	if(obstacle_distance > 0.5){
			avoidObstacle = false;
			cmd_vel_msg.linear.x = 0.5;
			cmd_vel_msg.angular.z = 0.0;
			if (robot_stopped == true){
				ROS_INFO("Moving Forward");
				robot_stopped = false;
			}
	}else{
			avoidObstacleFrente = true;
			
			if(readings[0] <= obstacle_distance){
				cmd_vel_msg.linear.x = 0.0;
				cmd_vel_msg.angular.z = 1.05;
				
			}
			else if(readings[1] <= obstacle_distance){
				
				cmd_vel_msg.linear.x = -0.2;
				cmd_vel_msg.angular.z = 1.05;
				
			}
			else if(readings[2] <= obstacle_distance){
				cmd_vel_msg.linear.x = 0.0;
				cmd_vel_msg.angular.z = -1.05;
			}
			/*else if(readings[3] <= obstacle_distance){
				cmd_vel_msg.linear.x = 0.0;
				cmd_vel_msg.angular.z = -1.05;
			}*/
			else{
				cmd_vel_msg.linear.x = 0.0;
				cmd_vel_msg.angular.z = 1.05;
			}
	}
	
	return cmd_vel_msg;
	
}
int main(int argc, char **argv){
	
	ros::init(argc,argv,"reactive_navigation");
	
	ros::NodeHandle n;
	
	//Publisher for /cmd_vel
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",20);
	//Subscriber for /base_scan
	ros::Subscriber laser_sub = n.subscribe("base_scan",100, laserCallback);
	
	ros::Rate loop_rate(10); // 10 hz
	
	geometry_msgs::Twist cmd_vel_msg;
	
	robot_stopped = true;
	obstacle_distance = 1;
	
	
	free(readings);
	
	while(ros::ok()){
		ROS_INFO("entrei");
		cmd_vel_msg = calculateCommand(readings);
		/*if(obstacle_distance > 0.5){
			cmd_vel_msg.linear.x = 0.5;
			cmd_vel_msg.angular.z = 0.0;
			if (robot_stopped == true){
				ROS_INFO("Moving Forward");
				robot_stopped = false;
			}
		}*/
		/* else {
			
			cmd_vel_msg.linear.x = 0.0;
			cmd_vel_msg.angular.z = 0.0;
			if(!robot_stopped){
				ROS_INFO("Stopping");
				robot_stopped = true;
			}
		}*/
		
		//publish velocity commands
		
		
		cmd_vel_pub.publish(cmd_vel_msg);
		ros::Time beginTime = ros::Time::now();
		ros::Time endTime = beginTime + ros::Duration();
		
		
		if(avoidObstacleFrente){
			while(ros::Time::now() < endTime){
				cmd_vel_pub.publish(cmd_vel_msg);
				ros::Duration(0.1).sleep();
			}
			avoidObstacleFrente = false;	
		}
		
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	
	return 0;
}
		
		
		
		
