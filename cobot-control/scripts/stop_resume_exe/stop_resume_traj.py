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
from std_msgs.msg import Bool

# misc imports
import yaml

# Threading imports
from threading import Thread

# importing eeg streaming 
from pylsl import StreamInlet, resolve_stream

# importing opencv packages
import cv2
import numpy as np

# importing process
from multiprocessing import Process
import time


def signal_handler(*args):
	print("Writing to file")
	with open("ERRP.txt", "w") as f:
		f.write("[ ")	
		for ERRP in ur5.ERRP:
			print(ERRP)
			f.write(str(ERRP) +" ")	
		f.write(" ]")	
		exit()
	

class InterruptableTrajectory(UR5Control):

	def __init__(self, planning_time=10, planner_id='RRT', z_safety_limit=0.01):
		super().__init__(planning_time=planning_time, planner_id=planner_id, z_safety_limit=z_safety_limit)
		rospy.init_node('stop_resume_exe_traj', anonymous=True)

		# initializing variables
		self.execution_start_time = 0
		self.new_traj_planned = False

		self.ERRP=list()

		self.traj_completed=False

		dir_path = RosPack().get_path('cobot-control') + "/scripts/stop_resume_exe/"
		
		# opening YAML file for topics
		with open(dir_path + "topics.yaml", 'r') as stream:
			try:
				self.topics = yaml.safe_load(stream)
			except yaml.YAMLError as exc:
				print(exc)
				print("Exiting because some issues with loading YAML")
				sys.exit(0)

		#Publisher
		self.is_started_pub = rospy.Publisher(self.topics['ur5_start'], Bool, queue_size=1)
		self.is_stopped_pub = rospy.Publisher(self.topics['ur5_stop'], Bool, queue_size=1)
		self.is_resumed_pub = rospy.Publisher(self.topics['ur5_resume'], Bool, queue_size=1)
		self.is_homed_pub = rospy.Publisher(self.topics['ur5_home'], Bool, queue_size=1)
				
		# Subscriber
		self.resumed_sub = rospy.Subscriber(self.topics['ur5_resume'], Bool, self.is_resumed_callback)
		self.started_sub = rospy.Subscriber(self.topics['ur5_start'], Bool, self.is_started_callback)
		self.stopped_sub = rospy.Subscriber(self.topics['ur5_stop'], Bool, self.is_stopped_callback)
		self.homed_sub = rospy.Subscriber(self.topics['ur5_home'], Bool, self.is_homed_callback)

		self.is_resumed = False
		self.is_started = False
		self.is_homed = False
		self.is_stopped = False

		self.flag_completed_traj=0 # flag is set when full trajectory is completed
		self.flag_moved_to_home=0 # flag is set when homed for the first time

		self.interr_robo_traj = moveit_msgs.msg.RobotTrajectory() 	# this is the message type that is published
		self.interr_traj = JointTrajectory() # interruptable trajectory
		self.new_waypoint = JointTrajectoryPoint() # new_waypoints for resuming trajectory

	def is_stopped_callback(self, data):
		self.is_stopped = data.data
		if self.is_stopped:
			self.move_group.stop()

	def is_resumed_callback(self, data):
		self.is_resumed = data.data

	def is_homed_callback(self, data):
		self.is_homed = data.data

	def is_started_callback(self, data):
		self.is_started = data.data

	def ext_trigger_stop_callback(self, data):
		temp_resumed=Bool()
		temp_started=Bool()
		temp_stopped=Bool()
		if self.is_started or self.is_resumed:
			temp_resumed.data=False
			temp_started.data = False
			temp_stopped.data = True
			self.is_started_pub.publish(temp_started)
			self.is_stopped_pub.publish(temp_stopped)
			self.is_resumed_pub.publish(temp_resumed)
			# self.move_group.stop()
		if not(self.traj_completed):
			self.ERRP[-1]=1	
		# print(self.ERRP)

	def ext_trigger_resume_callback(self, data):
		temp_resumed=Bool()
		temp_stopped=Bool()
		self.ERRP.append(0)
		# print(self.ERRP)
		if self.is_stopped:
			temp_resumed.data = True
			temp_stopped.data = False
			self.is_stopped_pub.publish(temp_stopped)
			self.is_resumed_pub.publish(temp_resumed)

	def ext_trigger_start_callback(self, data):
		self.ERRP.append(0)
		# print(self.ERRP)
		temp_homed=Bool()
		temp_started=Bool()
		if self.is_homed:
			temp_started.data = True
			temp_homed.data= False
			self.is_started_pub.publish(temp_started)
			self.is_homed_pub.publish(temp_homed)


	def lsl_streaming(self):

		# first resolve an EEG stream on the lab network
		print("looking for the digital input stream...")
		streams = resolve_stream('type', 'EEG')

		# create a new inlet to read from the stream
		inlet = StreamInlet(streams[0])
		prev=True
		while True:
			# get a new sample (you can also omit the timestamp part if you're not
			# interested in it)
			sample,_ = inlet.pull_sample()
			current=sample[3]
			if current!=prev:
				if current:
					self.ext_trigger_stop_callback(True)
				elif not(current) and self.is_homed:
					self.ext_trigger_start_callback(True)
				# elif not(current) and self.new_traj_planned==False:
				# 	self.ext_trigger_resume_callback(True)
			prev=current
			
			


	def generate_waypoints_0(self):
		# defining the waypoints
		initial_waypoints = []

		wpose = self.get_pose_from_yaml("start_pose")

		# Getting random waypoints (1-4 waypoints chosen randomly)
		count=random.randint(1,4)
		while(count>0):
			random_pose=self.get_random_pose_()
			while(random_pose.pose.position.z<0.01):
				random_pose=self.get_random_pose_()
			wpose.position.x=random_pose.pose.position.x
			wpose.position.y=random_pose.pose.position.y
			wpose.position.z=random_pose.pose.position.z
			initial_waypoints.append(copy.deepcopy(wpose))
			count-=1
		return initial_waypoints


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



	def execute_cartesian_traj_with_rand_wp(self):
		"""
		Searches for a random trajectory and executes it.
		Returns
		-------
		True if the trajectory was executed without any interruption, else false.
		"""
		
		# obtaining moveit's trajectory
		(continuous_traj, fraction) = self.get_cartesian_trajectory(self.generate_waypoints_0(), 0.01)
		while(fraction < 0.3):
			(continuous_traj, fraction) = self.get_cartesian_trajectory(self.generate_waypoints_0(), 0.01)
		print("Obtained MoveIt's trajectory that is: ", str(fraction*100), " percent complete.")
		self.interr_traj.header = continuous_traj.joint_trajectory.header
		self.interr_traj.joint_names = continuous_traj.joint_trajectory.joint_names

		# intiallly add all points to the trajectory
		self.interr_traj.points = continuous_traj.joint_trajectory.points
		self.interr_robo_traj.joint_trajectory = self.interr_traj
		self.new_traj_planned = True
		return self._send_traj_to_manipulator(self.interr_robo_traj)


	def store_resumed_cartesian_path(self):
		time_elapsed =   rospy.get_rostime() - self.execution_start_time
		num_waypoints = len(self.interr_robo_traj.joint_trajectory.points)
		print(num_waypoints)
		if(num_waypoints==0):
			print("No cartesian path planned")
		else:
			rospy.loginfo("Truncating the original trajectory for executing on resume...")
			for i in range(0, num_waypoints):
				dt = self.interr_robo_traj.joint_trajectory.points[i].time_from_start - time_elapsed
				if dt.secs >= 0.0 and dt.nsecs >= 0.0:
					rospy.loginfo("Found the closest waypoint at index: " + str(i))
					self.new_waypoint.positions = self.move_group.get_current_joint_values()
					self.interr_robo_traj.joint_trajectory.points.insert(i, self.new_waypoint)
					# truncate the waypoints
					self.interr_robo_traj.joint_trajectory.points = self.interr_robo_traj.joint_trajectory.points[i:]
					break
			# update the time at each waypoint
			num_waypoints = len(self.interr_robo_traj.joint_trajectory.points)
			for j in range(0, num_waypoints):
				self.interr_robo_traj.joint_trajectory.points[j].time_from_start -= dt


	def execute_interruptable_trajectory(self):
		while not rospy.is_shutdown():
			if self.flag_moved_to_home:
				if self.is_started:
					if(self.execute_cartesian_traj_with_rand_wp()):
						rospy.loginfo("Trajectory execution completed!")
						self.is_started=False
						self.new_traj_planned=True
						self.traj_completed=True
					else:
						self.store_resumed_cartesian_path()
						self.new_traj_planned = False
					self.flag_moved_to_home = 0

			# elif not(self.new_traj_planned) and self.is_resumed:
			# 	if (self._send_traj_to_manipulator(self.interr_robo_traj)):
			# 		rospy.loginfo("Trajectory execution completed!")
			# 		self.is_started=False
			# 		self.new_traj_planned=True
			# 		self.traj_completed=True
			# 	else:
			# 		self.store_resumed_cartesian_path()
			# 		self.new_traj_planned = False
			# 		self.flag_moved_to_home = 0

			elif self.is_homed and self.flag_moved_to_home==0:
				
				self.move_to_joint_state("home_joint_state")
				self.flag_moved_to_home = 1
				self.traj_completed=False

if __name__ == '__main__':
	signal.signal(signal.SIGINT, signal_handler)
	# signal.signal(signal.SIGTERM, signal_handler)

	ur5 = InterruptableTrajectory()

	# print("Please wait, configuring... \nMoving to start position...")
	# ur5.move_to_joint_state('start_joint_state')
	t1 = Thread(target = ur5.execute_interruptable_trajectory,args=())
	t2 = Thread(target = ur5.lsl_streaming,args=())

	# starting thread 2
	t2.start()

	# starting thread 1
	t1.start()


	# wait until thread 1 is completely executed
	t1.join()
	# wait until thread 2 is completely executed
	t2.join()
