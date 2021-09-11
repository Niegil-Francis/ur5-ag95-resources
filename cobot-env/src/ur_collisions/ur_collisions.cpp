#include "ur_collisions.h"

UrCollisions::UrCollisions(){
	gazebo::client::setup();

	// initializing communication with gazebo
	this->gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
	this->gz_node_->Init();
	this->contacts_sub_ = this->gz_node_->Subscribe("~/physics/contacts", &UrCollisions::contactsCallback, this);

	// initializing ROS communication
	this->nh_ = ros::NodeHandle();
	this->collision_pub_ = this->nh_.advertise<std_msgs::Int32>("/ur5_collided", 1);
}

void UrCollisions::contactsCallback(ConstContactsPtr &_contacts){
	static int cur_contact;
	int contacts=0;
	cur_contact = _contacts->contact_size() - 1;
	while(!(cur_contact < 0)){
		if(_contacts->contact(cur_contact).collision1().find("ur5") != std::string::npos){
			ROS_INFO_STREAM("Collision with UR5 detected!\n");
			contacts++;
			collision_msg_.data=contacts;
			this->collision_pub_.publish(this->collision_msg_);
			break;
		}
		if(_contacts->contact(cur_contact).collision2().find("ur5") != std::string::npos){
			ROS_INFO_STREAM("Collision with UR5 detected!\n");
			contacts++;
			collision_msg_.data=contacts;
			this->collision_pub_.publish(this->collision_msg_);
			break;
		}
		cur_contact--;
	}

}

void UrCollisions::stop(){
	gazebo::client::shutdown();
}

int main(int _argc, char **_argv){
	ros::init(_argc, _argv, "ur_collisions");
	UrCollisions ur_collisions;
	
	ros::Rate loop_rate(125);
	ros::AsyncSpinner spinner(10);
	spinner.start();

	ROS_INFO_STREAM("Press CTRL+C to end this node");

	while(ros::ok()){
		try{
			loop_rate.sleep();
		}
		catch(...){
			break;
		}
	}
	
	ur_collisions.stop();
	ros::shutdown();
	
	ROS_INFO_STREAM("Exiting.");
	return 0;
}