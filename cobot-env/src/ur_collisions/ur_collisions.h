#ifndef UR_COLLISIONS_H
#define UR_COLLISIONS_H

#include <ros/ros.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <std_msgs/Empty.h>



class UrCollisions{

public:
  /// @brief Constructor
  UrCollisions();

  /// @brief ROS node handle
  ros::NodeHandle nh_;

  /// @brief Gazebo transport node for communication
  gazebo::transport::NodePtr gz_node_;

  /// @brief Gazebo subscriber to contacts topic
  gazebo::transport::SubscriberPtr contacts_sub_;

  /// @brief ROS publisher of UR5 collision
  ros::Publisher collision_pub_;

  /// @brief Empty message to publish
  std_msgs::Empty collision_msg_;

  /**
   * @brief Callback function contacts
   * 
   * @param _contacts Reference to the Contacts object
   */ 
  void contactsCallback(ConstContactsPtr &_contacts);

  /**
   * @brief Shutdown the gazebo client
   */ 
  void stop();
};

#endif