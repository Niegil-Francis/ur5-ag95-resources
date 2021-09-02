#include "viman.h"

void Viman::initROSVars(ros::NodeHandle& node){
	isFlying = false;
	
	pubTakeOff = node.advertise<std_msgs::Empty>("/viman/takeoff",1024);
    pubLand = node.advertise<std_msgs::Empty>("/viman/land",1024);
    pubReset = node.advertise<std_msgs::Empty>("/viman/reset",1024);
    pubCmd = node.advertise<geometry_msgs::Twist>("/cmd_vel",1024); 
		
}

void Viman::toggle_ready(void){
	// Stop all the propellers
	Viman::allStop();
	if(isFlying){				
		pubLand.publish(std_msgs::Empty());
		ROS_INFO("Landing...");
    
		isFlying = false;
	}
	else{
		pubTakeOff.publish(std_msgs::Empty());
		ROS_INFO("Sending take-off ready signal...");
    
		isFlying = true;
	}
}

void Viman::allStop(void){
    if(!isFlying)
        return;
    
    twist_msg.linear.x=0;
    twist_msg.linear.y=0;
    twist_msg.linear.z=0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z= 0.0;
    
    pubCmd.publish(twist_msg);
}

void Viman::move(float speed_x, float speed_y, float speed_z){
    if (!isFlying)
		return;
     
    twist_msg.linear.x = speed_x;
    twist_msg.linear.y = speed_y;
    twist_msg.linear.z = speed_z;
    twist_msg.angular.x = sgn(speed_y)*0.05;
    twist_msg.angular.y = sgn(speed_x)*0.05;
    twist_msg.angular.z = 0.0;
    pubCmd.publish(twist_msg);
}

void Viman::move(float speed_x, float speed_y, float speed_z, float speed_yaw){
    if (!isFlying)
		return;
     
    twist_msg.linear.x = speed_x;
    twist_msg.linear.y = speed_y;
    twist_msg.linear.z = speed_z;
    twist_msg.angular.x = sgn(speed_y)*0.05;
    twist_msg.angular.y = sgn(speed_x)*0.05;
    twist_msg.angular.z = speed_yaw;
    pubCmd.publish(twist_msg);
}

void Viman::yaw(float speed){
	if (!isFlying)
		return;
    
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = speed;
    pubCmd.publish(twist_msg);
}

int Viman::sgn(float x){
	if (x > 0) return 1;
	if (x < 0) return -1;
	return 0;
}
