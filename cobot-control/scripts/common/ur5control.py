#!/usr/bin/python3

# generic imports
import sys
import time

# ROS imports
import rospy
from rospkg import RosPack

# MoveIt imports
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg

# message imports
from dh_gripper_msgs.msg import GripperState
import geometry_msgs.msg

# math related imports
import numpy as np
import tf
from tf.transformations import quaternion_matrix
from tf.transformations import translation_matrix
from math import dist, fabs, cos

# misc imports
import yaml

class UR5Control:
	"""
	A utility class containing methods that aid in moving the UR5 arm
	as well as the AG95 gripper. It also contains methods to estimate
	the next position to reach. It reads pose values from YAML file.
	"""

	def __init__(self, planning_time = 10,
                       planner_id = 'RRT',
                        z_safety_limit = 0.01):
		"""
		Initializes moveit_commander and related publishers and subscribers to control
        the UR5 robot. Inherit this class incase you wish to add custom functions as 
        per the applications requirements.

		Parameters
		----------
        * planning_time : int:
            Planning time for the motion planner
        * planner_id : str:
            Planner ID of the motion planner
		* z_safety_limit: float:
		        Minimum z coordinate of TCP wrt base_link frame (in m)
		"""
		moveit_commander.roscpp_initialize(sys.argv)
		
		self.gripper_pub = rospy.Publisher('gripper_pose_1', GripperState, queue_size=1)
		self.gripper_msg = GripperState()

		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		group_name = "manipulator"
		self.move_group = moveit_commander.MoveGroupCommander(group_name)
		self.move_group.set_planning_time(planning_time)
		self.move_group.set_planner_id(planner_id)

		print("Pausing (init)...")
		time.sleep(1)
		print("...done!")

		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
												moveit_msgs.msg.DisplayTrajectory,
												queue_size=20)
		self.z_safety_limit = z_safety_limit

		# initializing variables
		rp = RosPack()
		dir_path = rp.get_path('cobot-control') + "/scripts/common/"

		# opening YAML file for poses
		with open(dir_path + "poses.yaml", 'r') as stream:
			try:
				self.determined_poses = yaml.safe_load(stream)
			except yaml.YAMLError as exc:
				print(exc)
				print("Exiting because some issues with loading YAML")
				sys.exit(0)

	def is_within_tolerance(self, goal, actual, tolerance):
		"""
		Convenience method for testing if the values in two lists are within a tolerance of each other.
		For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
		between the identical orientations q and -q is calculated correctly).
		@param: goal	   A list of floats, a Pose or a PoseStamped
		@param: actual	 A list of floats, a Pose or a PoseStamped
		@param: tolerance  A float
		@returns: bool
		"""
		if type(goal) is list:
			for index in range(len(goal)):
				if abs(actual[index] - goal[index]) > tolerance:
					return False
		
		elif type(goal) is geometry_msgs.msg.PoseStamped:
			return self.is_within_tolerance(goal.pose, actual.pose, tolerance)

		elif type(goal) is geometry_msgs.msg.Pose:
			x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
			x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
			# Euclidean distance
			d = dist((x1, y1, z1), (x0, y0, z0))
			# phi = angle between orientations
			cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
			return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

		return True

	def get_base_link_coordinates(self, camera_coord):
		"""
		Returns the coordinate of the TCP wrt base_link frame from the camera frame coordinates.
		"""
		# translation
		trans = 0

		# quaternion representing the rotation
		quat = 0

		# object to listens to the transformations
		listener = tf.TransformListener()

		while not rospy.is_shutdown():
			try:
				# We obtain a vector describing the translation and a quaternion describing the rotation
				# as soon as something is returned, break the loop
				trans, quat = listener.lookupTransform('/base_link', '/camera_link2', rospy.Time(0)) 
				break
			except:
				continue
		
		# Obtain homogenous matrices from translation vector and the quaternion
		# Multiply the homogenous transformations to get a combined HT
		combinedHT = np.matmul(translation_matrix(trans), quaternion_matrix(quat))
		
		base_link_coord = np.matmul(np.array(combinedHT), camera_coord)
		
		return base_link_coord

	def get_pose_from_yaml(self, pose_id_name):
		"""
		Returns
		---
		Pose from the YAML file
		"""
		pose_yaml = self.move_group.get_current_pose().pose

		pose_yaml.position.x = self.determined_poses[pose_id_name]['position']['x']
		pose_yaml.position.y = self.determined_poses[pose_id_name]['position']['y']
		pose_yaml.position.z = self.determined_poses[pose_id_name]['position']['z']

		pose_yaml.orientation.x = self.determined_poses[pose_id_name]['orientation']['x']
		pose_yaml.orientation.y = self.determined_poses[pose_id_name]['orientation']['y']
		pose_yaml.orientation.z = self.determined_poses[pose_id_name]['orientation']['z']
		pose_yaml.orientation.w = self.determined_poses[pose_id_name]['orientation']['w']

		return pose_yaml

	def get_pick_goal_pose(self, base_link_coord):
		"""
		Converts the TCP coordinates in base_link frame to pick goal pose.
		A new parameter maybe added to this method to provide orientation information.

		Parameters
		----
		- base_coord : column vector describing the goal TCP coordinates wrt base_link frame

		Returns
		---
		Goal pose using input parameters
		"""
		pose_goal = self.move_group.get_current_pose().pose
		
		pose_goal.position.x = base_link_coord[0][0]
		pose_goal.position.y = base_link_coord[1][0]
		pose_goal.position.z = base_link_coord[2][0]

		pose_goal.orientation.x = self.determined_poses['pick_orientation']['orientation']['x']
		pose_goal.orientation.y = self.determined_poses['pick_orientation']['orientation']['y']
		pose_goal.orientation.z = self.determined_poses['pick_orientation']['orientation']['z']
		pose_goal.orientation.w = self.determined_poses['pick_orientation']['orientation']['w']

		return pose_goal

	def get_random_pose_(self):
		random_pose=self.move_group.get_random_pose()
		return random_pose
		
	def get_cartesian_trajectory(self, waypoints, eef_step):
		"""
		Parameters
		----------

		waypoints: list of waypoints
        eef_step: step size of EE (in m)

		Returns
		-------
		
		* plan: trajectory of the cartesian path between the waypoints
		* fraction: fraction of the trajectory planned between the current and the end 
		goal.
		"""
		(plan, fraction) = self.move_group.compute_cartesian_path(
			waypoints, eef_step, 0.0
		)

		return plan, fraction

	def control_gripper(self, target_force, target_position):
		"""
		Controls the gripper

		Parameters
		---------
		target_force: float:
			Force applied on the fingers
		target_position: float:
			Position of the fingers
		"""
		if not rospy.is_shutdown():
			self.gripper_msg.target_position = target_position
			self.gripper_msg.target_force = target_force

			self.gripper_pub.publish(self.gripper_msg)
			
			# after sending the gripper command, sleep for 2 seconds
			print("Pausing (control_gripper)...")
			time.sleep(2)
			print("...done!")

	def open_gripper(self, target_force=5):
		"""
		Utility function that opens the gripper

		Parameters
		---------
		target_force: float:
			Force applied on the fingers
		"""
		self.control_gripper(target_force=target_force, target_position=1000)
	
	def close_gripper(self, target_force=5):
		"""
		Utility function that closes the gripper

		Parameters
		---------
		target_force: float:
			Force applied on the fingers
		"""
		self.control_gripper(target_force=target_force, target_position=20)
	
	def move_to_location(self, goal_pose):
		"""
		Move the TCP to the goal pose. Checks for Z safety limit

		Parameters
		----------
		- goal_pose: pose to which path must be planned

		Return
		------
		- idx: idx can take various values depending on the following conditions: 
			- 0: User does not agree to execute the planned path
			- 1: User agrees to execute the planned path and the path is executed
		"""
		self.move_group.clear_pose_targets()
		current_pose = self.move_group.get_current_pose().pose
		print("Planning path from pose (current pose):  ")
		print(current_pose)

		if goal_pose.position.z < self.z_safety_limit:
			goal_pose.position.z = self.z_safety_limit
			rospy.logwarn("The goal pose has invalid Z coordinate for TCP. Coordinate overridden to Z = 0.01")

		print("To pose (goal pose): ")
		print(goal_pose)

		self.move_group.set_pose_target(goal_pose)

		plan = self.move_group.plan()
		
		print("Planning the path. Check RViz for visualization.")
		user_in = input('Do you want to execute the path on the real cobot? [y|n] ')
		# user_in = 'y'
		
		if user_in == 'y':
			print("Executing path after 3 seconds...")
			rospy.sleep(3)
			self.move_group.go(wait=True)
			self.move_group.stop()
			self.move_group.clear_pose_targets()
			return 1
		else:
			print("Path no executed!")
			self.move_group.clear_pose_targets()
			return 0

	def move_to_joint_state(self, goal_state_id):
		"""
		Move the joints to goal_state. The joint values are taken from 'poses.yaml'
		file

		Returns
		-------
		True - if path executed and the final pose is in tolerance limits
		False - if path executed and the final pose is NOT in tolerance limits
		2 - if path not executed
		"""
		joint_goal = self.move_group.get_current_joint_values()
		for i in range(0, 6):
			joint_goal[i] = self.determined_poses[goal_state_id][i]
		
		self.move_group.set_joint_value_target(joint_goal)
		self.move_group.plan()

		print("Planning the path. Check RViz for visualization.")
		# user_in = input('Do you want to execute the path on the real cobot? [y|n] ')
		user_in = 'y'
		if user_in == 'y':
			print("Executing path after 3 seconds...")
			rospy.sleep(3)
			self.move_group.go(wait=True)
			self.move_group.stop()
			self.move_group.clear_pose_targets()
			return self.is_within_tolerance(joint_goal, self.move_group.get_current_joint_values(), 0.01)
		else:
			print("Path no executed!")
			self.move_group.clear_pose_targets()
			return 2
