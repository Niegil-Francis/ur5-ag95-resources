#include "plugin_altimeter.h"

#include <cmath>
#include <stdlib.h>
#include <iostream>

namespace gazebo {

// Contructor
Altimeter::Altimeter(){}

// Destructor
Altimeter::~Altimeter(){
  node_handle_->shutdown();
  delete node_handle_;
}

// Load the plugin
void Altimeter::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  
  world = _model->GetWorld();
  link = _model->GetLink();
  link_name_ = link->GetName();
  
  if(_sdf->HasElement("bodyName")){
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link = _model->GetLink(link_name_);
  }
  
  if (!link){
    ROS_FATAL("plugin_altimeter error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }
  
  // default parameters
  frame_id_ = link->GetName();
  altimeter_topic_ = "/human_drone/height";
  zeroErr_topic_ = "/human_drone/setZero";
  noise_ = 0.01;
  
  // load parameters
  if (_sdf->HasElement("frameId"))            frame_id_ = _sdf->GetElement("frameId")->Get<std::string>();
  if (_sdf->HasElement("altimeterTopicName")) altimeter_topic_ = _sdf->GetElement("altimeterTopicName")->Get<std::string>();
  if (_sdf->HasElement("setZeroTopicName")) zeroErr_topic_ = _sdf->GetElement("setZeroTopicName")->Get<std::string>();
  if (_sdf->HasElement("noise"))     	  noise_ = _sdf->GetElement("noise")->Get<double>();  
  height_.header.frame_id = frame_id_;

  // Make sure the ROS node for Gazebo has already been initialized                 
  if (!ros::isInitialized()){
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle;

  // advertise altimeter
  if(!altimeter_topic_.empty()){
    altimeter_publisher_ = node_handle_->advertise<geometry_msgs::PointStamped>(altimeter_topic_, 1024);
  }
  
  // subscribe command: zero error command
  ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Empty>(
      zeroErr_topic_, 1,
      boost::bind(&Altimeter::ZeroErrCallbck, this, _1),
      ros::VoidPtr(), &callback_queue_);
    zeroErr_subscriber_ = node_handle_->subscribe(ops);
      
  // connect Update function
  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Altimeter::Update, this));
  
  ROS_INFO("Loaded Altimeter plugin.");
  
}

void Altimeter::Reset(){}

// Update
void Altimeter::Update(){

	// Get new command
	callback_queue_.callAvailable();
	
#if (GAZEBO_MAJOR_VERSION >= 8)
  common::Time sim_time = world->SimTime();
#else
  common::Time sim_time = world->GetSimTime();
#endif

#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Pose3d pose = link->WorldPose();
  double height = pose.Pos().Z();
#else
  math::Pose pose = link->GetWorldPose();
  double height = pose.pos.z;
#endif
	
	// add noise to the actual value
	height += noise_*drand48();
	
	// remove zero error
	height -= zero_err_;
	
    height_.header.stamp = ros::Time::now();
    height_.point.z = height;
    altimeter_publisher_.publish(height_);
}

// Callback function to set zero error value
void Altimeter::ZeroErrCallbck(const std_msgs::EmptyConstPtr& msg){
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Pose3d pose = link->WorldPose();
  zero_err_ = pose.Pos().Z()+noise_*drand48();
#else
  math::Pose pose = link->GetWorldPose();
  zero_err_ = pose.pos.z+noise_*drand48();
#endif
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Altimeter)

} // namespace gazebo
