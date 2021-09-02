#ifndef PLUGIN_ALTIMETER_H
#define PLUGIN_ALTIMETER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#if (GAZEBO_MAJOR_VERSION == 11)
  #include <ignition/math6/ignition/math.hh>
#elif (GAZEBO_MAJOR_VERSION == 9)
  #include <ignition/math4/ignition/math.hh>
#endif

#include <std_msgs/Empty.h>
#include <geometry_msgs/PointStamped.h>


namespace gazebo{

class Altimeter : public ModelPlugin{
public: 
  Altimeter();
  virtual ~Altimeter();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  physics::WorldPtr world;

  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;

  ros::CallbackQueue callback_queue_;  
  ros::Publisher altimeter_publisher_;
  ros::Subscriber zeroErr_subscriber_;
  
  void ZeroErrCallbck(const std_msgs::EmptyConstPtr&);
  
  geometry_msgs::PointStamped height_;

  std::string altimeter_topic_;
  std::string zeroErr_topic_;
  std::string link_name_;
  std::string frame_id_;

  double noise_;		// std dev of additive Gaussian noise
  double zero_err_ = 0;	// to set a particular height as reference

  event::ConnectionPtr updateConnection;
};

}

#endif // PLUGIN_ALTIMETER_H
