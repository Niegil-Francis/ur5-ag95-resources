#ifndef FLIGHT_CTRL
#define FLIGHT_CTRL

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

/**
 * @brief Class to send the commands to human_drone
 */

class Viman{
protected:    
    // Utility functions
	int sgn(float);
	
public:
    
    bool isFlying;
    geometry_msgs::Twist twist_msg;
    
    ros::Publisher pubTakeOff;
	ros::Publisher pubLand;
	ros::Publisher pubReset;
	ros::Publisher pubCmd;
    
    Viman(){}
        
    Viman(ros::NodeHandle& node){
        initROSVars(node);
    }
	    
    void initROSVars(ros::NodeHandle& node);
    
    // Functions to control viman state
	void toggle_ready(void);
	void allStop(void);
	void move(float speed_x, float speed_y, float speed_z, float speed_yaw);
	void move(float speed_x, float speed_y, float speed_z);
	void yaw(float);
    
};

#endif // FLIGHT_CTRL
