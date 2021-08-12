#!/usr/bin/python3

# Adding the common python files to path
from os import wait
import random
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
import yaml

def signal_handler():
	sys.exit(0)

class InterruptableTrajectory(UR5Control):

	def __init__(self, planning_time=10, planner_id='RRT', z_safety_limit=0.01):
		super().__init__(planning_time=planning_time, planner_id=planner_id, z_safety_limit=z_safety_limit)
		rospy.init_node('stop_resume_exe_traj', anonymous=True)

		# initializing variables
		self.execution_start_time = 0
		self.new_traj_planned = False

		dir_path = RosPack().get_path('cobot-control') + "/scripts/stop_resume_exe/"
		
		# opening YAML file for topics
		with open(dir_path + "topics.yaml", 'r') as stream:
			try:
				self.topics = yaml.safe_load(stream)
			except yaml.YAMLError as exc:
				print(exc)
				print("Exiting because some issues with loading YAML")
				sys.exit(0)
				
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


	def generate_waypoints_0(self):
		# defining the waypoints
		initial_waypoints = []

		wpose = self.get_pose_from_yaml("start_pose")

		count=random.randint(1,4)
		while(count>0):
			random_pose=self.get_random_pose_()
			while(random_pose.pose.position.z<0.190):
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

	def execute_interruptable_trajectory(self):
		self.new_traj_planned = True
		interr_robo_traj = moveit_msgs.msg.RobotTrajectory() 	# this is the message type that is published
		interr_traj = JointTrajectory()
		new_waypoint = JointTrajectoryPoint()

	
		while not rospy.is_shutdown():
			if self.flag_moved_to_home:
				if self.is_started:
					# obtaining moveit's trajectory
					(continuous_traj, fraction) = self.get_cartesian_trajectory(self.generate_waypoints_0(), 0.01)
					while(fraction < 0.3):
						(continuous_traj, fraction) = self.get_cartesian_trajectory(self.generate_waypoints_0(), 0.01)
					print("Obtained MoveIt's trajectory that is: ", str(fraction*100), " percent complete.")
					interr_traj.header = continuous_traj.joint_trajectory.header
					interr_traj.joint_names = continuous_traj.joint_trajectory.joint_names
			
					# intiallly add all points to the trajectory
					interr_traj.points = continuous_traj.joint_trajectory.points
					interr_robo_traj.joint_trajectory = interr_traj
					if (self._send_traj_to_manipulator(interr_robo_traj)):
						rospy.loginfo("Trajectory execution completed!")
						self.is_started=False
						self.new_traj_planned=True
					else:
						time_elapsed =   rospy.get_rostime() - self.execution_start_time
						num_waypoints = len(interr_robo_traj.joint_trajectory.points)
						rospy.loginfo("Truncating the original trajectory for executing on resume...")
						for i in range(0, num_waypoints):
							dt = interr_robo_traj.joint_trajectory.points[i].time_from_start - time_elapsed
							if dt.secs >= 0 and dt.nsecs >= 0:
								rospy.loginfo("Found the closest waypoint at index: " + str(i))
								new_waypoint.positions = self.move_group.get_current_joint_values()
								interr_robo_traj.joint_trajectory.points.insert(i, new_waypoint)
								# truncate the waypoints
								interr_robo_traj.joint_trajectory.points = interr_robo_traj.joint_trajectory.points[i:]
								
								# update the time at each waypoint
								num_waypoints = len(interr_robo_traj.joint_trajectory.points)
								for i in range(0, num_waypoints):
									interr_robo_traj.joint_trajectory.points[i].time_from_start -= dt
						self.new_traj_planned = False
					self.flag_moved_to_home = 0
				elif self.is_stopped:
					self.flag_moved_to_home=0
			elif not(self.new_traj_planned) and self.is_resumed:
				if (self._send_traj_to_manipulator(interr_robo_traj)):
					rospy.loginfo("Trajectory execution completed!")
					self.is_started=False
					self.new_traj_planned=True
				else:
					self.new_traj_planned = False
					time_elapsed =   rospy.get_rostime() - self.execution_start_time
					num_waypoints = len(interr_robo_traj.joint_trajectory.points)
					rospy.loginfo("Truncating the original trajectory for executing on resume...")
					for i in range(0, num_waypoints):
						dt = interr_robo_traj.joint_trajectory.points[i].time_from_start - time_elapsed
						if dt.secs >= 0 and dt.nsecs >= 0:
							rospy.loginfo("Found the closest waypoint at index: " + str(i))
							new_waypoint.positions = self.move_group.get_current_joint_values()
							interr_robo_traj.joint_trajectory.points.insert(i, new_waypoint)
							# truncate the waypoints
							interr_robo_traj.joint_trajectory.points = interr_robo_traj.joint_trajectory.points[i:]
							
							# update the time at each waypoint
							num_waypoints = len(interr_robo_traj.joint_trajectory.points)
							for i in range(0, num_waypoints):
								interr_robo_traj.joint_trajectory.points[i].time_from_start -= dt
			elif self.is_homed and self.flag_moved_to_home==0:
				self.move_to_joint_state("home_joint_state")
				self.flag_moved_to_home = 1

if __name__ == '__main__':
	signal.signal(signal.SIGINT, signal_handler)
	signal.signal(signal.SIGTERM, signal_handler)

	ur5 = InterruptableTrajectory()

	# print("Please wait, configuring... \nMoving to start position...")
	# ur5.move_to_joint_state('start_joint_state')

	ur5.execute_interruptable_trajectory()