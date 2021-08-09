#!/usr/bin/python3

# Adding the common python files to path
from os import wait
import sys
from rospkg import RosPack
sys.path.insert(1, RosPack().get_path('cobot-control') + "/scripts/common/")
from ur5control import UR5Control

import signal
import copy

# ROS imports
import rospy
from rospkg import RosPack

# MoveIt imports
import moveit_msgs.msg

# message imports
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool

# misc imports
import numpy as np
import yaml

def signal_handler():
	sys.exit(0)

class InterruptableTrajectory(UR5Control):

	def __init__(self, planning_time=10, planner_id='RRT', z_safety_limit=0.01):
		super().__init__(planning_time=planning_time, planner_id=planner_id, z_safety_limit=z_safety_limit)
		rospy.init_node('stop_resume_exe_traj', anonymous=True)

		# initializing variables
		dir_path = RosPack().get_path('cobot-control') + "/scripts/stop_resume_exe/"
		
		# opening YAML file for topics
		with open(dir_path + "topics.yaml", 'r') as stream:
			try:
				self.topics = yaml.safe_load(stream)
			except yaml.YAMLError as exc:
				print(exc)
				print("Exiting because some issues with loading YAML")
				sys.exit(0)
				
		self.resumed_sub = rospy.Subscriber(self.topics['ur5_resumed'], Bool, self.is_resumed_callback)

		self.is_resumed = False

	def is_resumed_callback(self, data):
		self.is_resumed = data.data
		if not self.is_resumed:
			self.move_group.stop()

	def generate_waypoints_0(self):
		# defining the waypoints
		initial_waypoints = []

		wpose = self.move_group.get_current_pose().pose
		wpose.position.z +=  0.1 
		wpose.position.y +=  0.2
		initial_waypoints.append(copy.deepcopy(wpose))

		wpose.position.x +=  0.2
		initial_waypoints.append(copy.deepcopy(wpose))

		return initial_waypoints

	def execute_interruptable_trajectory(self, continuous_traj):
		interr_robo_traj = moveit_msgs.msg.RobotTrajectory() 	# this is the message type that is published
		interr_traj = JointTrajectory()
		interr_traj.header = continuous_traj.joint_trajectory.header
		interr_traj.joint_names = continuous_traj.joint_trajectory.joint_names
		
		# intiallly add all points to the trajectory
		interr_traj.points = continuous_traj.joint_trajectory.points
		interr_robo_traj.joint_trajectory = interr_traj

		while not rospy.is_shutdown():
			if self.is_resumed:
				if (self.move_group.execute(interr_robo_traj, wait=True)):
					rospy.loginfo("Trajectory execution completed!")
					break
				else:
					rospy.loginfo("Truncating the original trajectory for executing on resume...")

					# calculating the euclidean distance to find the next waypoint in original trajectory.
					# the next waypoint has the smallest euclidean distance from current pose
					# min_dist = 0
					# min_dist_idx = 0
					# next_waypoint = 0
					# num_waypoints = len(continuous_traj.joint_trajectory.points)
					# current_pose = self.move_group.get_current_joint_values()

					# for i in range(0, num_waypoints):
						



					# break
		
		# cur_point = JointTrajectoryPoint()
		# next_point = JointTrajectoryPoint()

		# num_waypoints = len(continuous_traj.joint_trajectory.points)
		# cur_waypoint = 1

		# cur_point.velocities = []
		# cur_point.accelerations = []
		# cur_point.time_from_start = rospy.Duration(0.0)

		# while True and not rospy.is_shutdown():
		# 	if self.is_resumed:
		# 		# add current position as first waypoint
		# 		cur_point.positions = self.move_group.get_current_joint_values()
		# 		interr_traj.points.append(cur_point)

		# 		# add next waypoint
		# 		next_point.positions = continuous_traj.joint_trajectory.points[cur_waypoint].positions
		# 		next_point.time_from_start = rospy.Duration(0.0000001)
		# 		interr_traj.points.append(next_point)
				
		# 		interr_robo_traj.joint_trajectory = interr_traj
		# 		self.move_group.execute(interr_robo_traj, wait=True)
		# 		interr_traj.points.clear()

		# 		cur_waypoint = cur_waypoint + 1
		# 		if cur_waypoint == num_waypoints:
		# 			break

if __name__ == '__main__':
	signal.signal(signal.SIGINT, signal_handler)
	signal.signal(signal.SIGTERM, signal_handler)

	ur5 = InterruptableTrajectory()

	print("Please wait, configuring... \nMoving to start position...")
	ur5.move_to_joint_state('start_joint_state')

	# obtaining moveit's trajectory
	(ur5_traj, fraction) = ur5.get_cartesian_trajectory(ur5.generate_waypoints_0(), 0.01)
	print("Obtained MoveIt's trajectory that is: ", str(fraction*100), " percent complete.")

	# ur5.move_group.execute(ur5_traj, wait=True)

	ur5.execute_interruptable_trajectory(ur5_traj)