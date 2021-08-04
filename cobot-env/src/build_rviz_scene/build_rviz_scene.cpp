// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rvt = rviz_visual_tools;

/// @brief Planning group name
static const std::string PLANNING_GROUP_NAME = "manipulator";

/// @brief Name of the package containing the resources
static const std::string PACKAGE_NAME = "cobot-env";

/// @brief Location of the table relative to the ROS package
static const std::string TABLE_LOCATION = "/meshes/table_complete.STL";


/**
 * @brief Add table to the RViz scene using moveit_visual_tools
 * 
 * @return True if successfully the table was published.
 */ 
bool addTableToScene(){
	moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
	visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("world", "/random/monitored_planning_scene"));
	visual_tools->loadPlanningSceneMonitor();
	visual_tools->loadMarkerPub();
	ros::Duration(2.0).sleep();
	visual_tools->setManualSceneUpdating();
	visual_tools->enableBatchPublishing();

	visual_tools->deleteAllMarkers();
	visual_tools->removeAllCollisionObjects();
	visual_tools->triggerPlanningSceneUpdate();

	// Initialize the pose with identity transformation
	Eigen::Isometry3d table_pose = Eigen::Isometry3d::Identity();
	
	// rotating the table_pose about X axis by 90 degs
	table_pose = table_pose * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX());


	std::string file_path = "file://" + ros::package::getPath(PACKAGE_NAME);
	if (file_path == "file://"){
    	ROS_FATAL_STREAM_NAMED("build_rviz_scene", 
		"Unable to get " << PACKAGE_NAME << " package path ");
	}

	file_path.append(TABLE_LOCATION);

	bool result = visual_tools->publishCollisionMesh(visual_tools->convertPose(table_pose), 
										"table",
										file_path, 
										rvt::BROWN);
	
	visual_tools->trigger();

	return result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "build_rviz_scene");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP_NAME);
  group.setPlanningTime(45.0);

  if(addTableToScene()){
  	ROS_INFO_STREAM("Added table to the RViz scene successfully!");
  }
  else{
	  ROS_WARN("Could not add table to the Rviz scene");
  }

  ros::shutdown();
  return 0;
}