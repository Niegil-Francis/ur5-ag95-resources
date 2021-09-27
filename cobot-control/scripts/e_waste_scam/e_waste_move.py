#!/usr/bin/python3

# Adding the common python files to path
from os import wait
import random
import sys
from rospkg import RosPack
sys.path.insert(1, RosPack().get_path('cobot-control') + "/scripts/common/")
from ur5control import UR5Control
from actionlib_msgs.msg import GoalID
import pandas as pd

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
from ur5tack.msg import URJoints

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

# Multithreading 
from threading import Thread
# Plotting
import matplotlib.pyplot as plt

from tf import TransformListener
from mpl_toolkits.mplot3d import Axes3D 
	
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Int16MultiArray

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
		self.tf = TransformListener()
		self.joint_names = ['upper_arm_link','shoulder_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link']
		self.joint_pos=[]
		self.jv = []
		self.done=False
		self.joint_goal=[]
		self.pub=rospy.Publisher('/joint_goal',URJoints,queue_size=1)
		self.dist=[]
		# self.sub = rospy.Subscriber('/joint_states', JointState, self.getCurrentJointPoses)

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
		
	def real_time_plotting(self):
		plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
		ax = plt.axes(projection='3d')
		for count,j in enumerate(self.joint_names):
			x = np.array([self.joint_pos[k][0] for k in [i+1 for i,val in enumerate(self.joint_pos) if(j in val)]])
			y = np.array([self.joint_pos[k][1] for k in [i+1 for i,val in enumerate(self.joint_pos) if(j in val)]])
			z = np.array([self.joint_pos[k][2] for k in [i+1 for i,val in enumerate(self.joint_pos) if(j in val)]])
			dist=[np.linalg.norm(np.array(i)-np.array(self.joint_goal)) for i in self.jv]
			scatter=ax.scatter(x, y, z, c = dist, s=1)
			ax.plot(x,y,z,label=j,linewidth=0.5)
		legend1 = ax.legend(*scatter.legend_elements(num=5),
		loc="upper right", title="Euclidean Distance")
		ax.legend(loc='upper left')
		ax.set_xlabel("x(m)")
		ax.set_ylabel('y(m)')
		ax.set_zlabel('z(m)')
		ax.add_artist(legend1)
		plt.show()
		# plt.savefig('Non_ERRP_traj.png')

	def getCurrentJointPoses(self):
		while(self.done==False):
			try:
				for i in self.joint_names:
					# Current end effector pose relative to the base_link frame
					self.tf.waitForTransform("/base_link",i,rospy.Time(),rospy.Duration(10))
					t = self.tf.getLatestCommonTime("/base_link", i) 
					position, quaternion = self.tf.lookupTransform("/base_link",i,t)
					self.joint_pos.append(i)
					self.joint_pos.append(position)
				curr=self.move_group.get_current_joint_values()
				self.dist.append(np.linalg.norm(np.array(curr)-np.array(self.joint_goal)))
			except:
				print("exception occured")
				pass
			# self.real_time_plotting()
			time.sleep(0.1)



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
			break_cntr=0
			while(fraction < 1):
				(continuous_traj, fraction) = self.get_cartesian_trajectory(wps[i], 0.01)
				break_cntr+=1
				if(break_cntr==50):
					print("Trajectory not found")
					sys.exit(0)
			print("Obtained MoveIt's trajectory that is: ", str(fraction*100), " percent complete.")
			self.interr_traj.header = continuous_traj.joint_trajectory.header
			self.interr_traj.joint_names = continuous_traj.joint_trajectory.joint_names
			self.joint_goal=continuous_traj.joint_trajectory.points[-1].positions
			test = URJoints()
			test.header.stamp = rospy.Time.now()
			test.base.data=self.joint_goal[2]
			test.elbow.data=self.joint_goal[0]
			test.shoulder.data=self.joint_goal[1]
			test.wrist1.data=self.joint_goal[3]
			test.wrist2.data=self.joint_goal[4]
			test.wrist3.data=self.joint_goal[5]
			self.pub.publish(test)
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
		self.done=True
			


if __name__ == '__main__':

	ur5 = InterruptableTrajectory()
	# time.sleep(5)
	# ur5.execute_cartesian_traj_with_wp()

	t1 = Thread(target = ur5.execute_cartesian_traj_with_wp,args=())
	t2 = Thread(target = ur5.getCurrentJointPoses,args=())

	# 	# starting thread 2
	t2.start()

	# 	# starting thread 1
	t1.start()


	# 	# wait until thread 1 is completely executed
	t1.join()
	# 	# wait until thread 2 is completely executed
	t2.join()


	# names=['upper_arm_link','shoulder_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link']
	names_axis=['upper_arm_link_x','upper_arm_link_y','upper_arm_link_z','shoulder_link_x','shoulder_link_y','shoulder_link_z',
	'forearm_link_x','forearm_link_y','forearm_link_z','wrist_1_link_x','wrist_1_link_y','wrist_1_link_z','wrist_2_link_x','wrist_2_link_y','wrist_2_link_z'
	,'wrist_3_link_x','wrist_3_link_y','wrist_3_link_z','distance']
	data=pd.DataFrame(columns=names_axis)
	fig = plt.figure()
	ax = plt.axes(projection='3d')
	for count,j in enumerate(ur5.joint_names):
		# print(count,j)
		x = np.array([ur5.joint_pos[k][0] for k in [i+1 for i,val in enumerate(ur5.joint_pos) if(j in val)]])
		y = np.array([ur5.joint_pos[k][1] for k in [i+1 for i,val in enumerate(ur5.joint_pos) if(j in val)]])
		z = np.array([ur5.joint_pos[k][2] for k in [i+1 for i,val in enumerate(ur5.joint_pos) if(j in val)]])
		# print(np.shape(x))
		data[names_axis[count*3]]=pd.Series(x)
		data[names_axis[count*3+1]]=pd.Series(y)
		data[names_axis[count*3+2]]=pd.Series(z)
		# scatter=ax.scatter(x, y, z, c = ur5.dist, s=1)
		# ax.plot(x,y,z,label=j,linewidth=0.5)
	# print("Size",np.shape(ur5.dist))
	data[names_axis[-1]]=pd.Series(ur5.dist)
	# print(data.head())
	# data.to_csv("Graph_data.csv")
	# # legend1 = ax.legend(*scatter.legend_elements(num=5),
	# # loc="upper right", title="Euclidean Distance")
	# ax.legend(loc='upper left')
	# ax.set_xlabel("x(m)")
	# ax.set_ylabel('y(m)')
	# # ax.set_zlabel('z(m)')
	# # ax.add_artist(legend1)
	# plt.savefig('Non_ERRP_traj.png')




