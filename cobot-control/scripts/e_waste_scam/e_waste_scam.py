#!/usr/bin/python3

# Adding the common python files to path
from os import wait
import random
import sys
from rospkg import RosPack
sys.path.insert(1, RosPack().get_path('cobot-control') + "/scripts/common/")
from ur5control import UR5Control
from actionlib_msgs.msg import GoalID

import signal
import copy

# ROS imports
import rospy
from rospkg import RosPack

# MoveIt imports
import moveit_msgs.msg

# message imports
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

# misc imports
import yaml

# Threading imports
from threading import Thread

# importing eeg streaming 
from pylsl import StreamInlet, resolve_stream

# importing opencv packages
import numpy as np

# importing process
from multiprocessing import Process
import time
import datetime
import sys
sys.path.append(".")
from poses import Poses
	

class InterruptableTrajectory(UR5Control):

	def __init__(self, planning_time=10, planner_id='RRT', z_safety_limit=0.01):
		super().__init__(planning_time=planning_time, planner_id=planner_id, z_safety_limit=z_safety_limit)
		rospy.init_node('e_waste_traj_follow', anonymous=True)
		self.Pose_class=Poses()
		# initializing variables
		self.execution_start_time = 0
		self.new_traj_planned = False

		self.ERRP=list()
		self.timestamp=list()

		self.traj_completed=False
	
		self.flag_completed_traj=0 # flag is set when full trajectory is completed

		self.interr_robo_traj = moveit_msgs.msg.RobotTrajectory() 	# this is the message type that is published
		self.interr_traj = JointTrajectory() # interruptable trajectory
		self.new_waypoint = JointTrajectoryPoint() # new_waypoints for resuming trajectory
		self.gripper=[]

		

	def generate_waypoints_0(self):
		# defining the waypoints
		wps = {}
		count=0
		self.gripper=[]
		pose_order=['home','open','eth1_up','eth1_pick','close','eth1_up', 'eth3_up', 'batt1_up',
					'bin_white', 'open', 'batt1_up', 'eth3_up','eth3_pick','close','eth3_up','open',
					'eth2_up','eth2_pick','close','eth2_up','bin_green', 'bin_green_low','open', 'bin_green',
					'dev_1', 'dev_2', 'eth5_pick', 'close' ,'eth5_up','open', 'home']

		for j,i in enumerate(pose_order):
			self.gripper.append(0)
			if(i=='close'):
				self.gripper[-1]=1
				count+=1 
				continue
			elif(i=='open'):
				count+=1
				continue
			else:
				pose=self.Pose_class.pose_call(i)
				try:
					wps[count].append(copy.deepcopy(pose))
				except:
					wps[count]=[]
					wps[count].append(copy.deepcopy(pose))
		return wps


	def _send_traj_to_manipulator(self, robo_traj):
		"""
		Sends the trajectory execution command to the manipulator and starts the timer.

		Parameters
		----------
		robo_traj : RobotTrajectory: The trajectory to be executed

		Returns
		-------
		True if the trajectory was executed without any interruption, else false.
		"""
		self.execution_start_time = rospy.get_rostime()
		return self.move_group.execute(robo_traj, wait=True)



	def execute_cartesian_traj_with_wp(self):
		"""
		Searches for a random trajectory and executes it.
		Returns
		-------
		True if the trajectory was executed without any interruption, else false.
		"""
		toggle_grip=1
		wps=self.generate_waypoints_0()
		
		for i in wps:
			if(wps[i][0]==self.Pose_class.pose_call('home')):
				self.move_to_joint_state('home_joint_state')
				toggle_grip=1
				self.open_gripper()
				continue
			
			# obtaining moveit's trajectory
			(continuous_traj, fraction) = self.get_cartesian_trajectory(wps[i], 0.09)
			while(fraction < 1):
				(continuous_traj, fraction) = self.get_cartesian_trajectory(wps[i], 0.01)
			print("Obtained MoveIt's trajectory that is: ", str(fraction*100), " percent complete.")
			self.interr_traj.header = continuous_traj.joint_trajectory.header
			self.interr_traj.joint_names = continuous_traj.joint_trajectory.joint_names
			print(continuous_traj.joint_trajectory.points)
			# intiallly add all points to the trajectory
			self.interr_traj.points = continuous_traj.joint_trajectory.points
			self.interr_robo_traj.joint_trajectory = self.interr_traj
			self.new_traj_planned = True
			if(self._send_traj_to_manipulator(self.interr_robo_traj)):
				if(toggle_grip):
					self.close_gripper()
					toggle_grip=0
				else:
					self.open_gripper()
					toggle_grip=1
					
				time.sleep(3)
				continue
			else:
				print("Trajectory not fully executted")
				sys.exit(0)
			


if __name__ == '__main__':

	ur5 = InterruptableTrajectory()
	time.sleep(5)

	# print("Please wait, configuring... \nMoving to start position...")
	# ur5.move_to_joint_state('start_joint_state')
	try:
		ur5.execute_cartesian_traj_with_wp()
	except KeyboardInterrupt:
		ur5.move_group.stop()
		sys.exit(1)
		

