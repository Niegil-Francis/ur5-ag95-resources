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
				
		self.resumed_sub = rospy.Subscriber(self.topics['ur5_resumed'], Bool, self.is_resumed_callback)

		self.is_resumed = False

		self.move_joint("elbow_joint")

	def is_resumed_callback(self, data):
		self.is_resumed = data.data
		if not self.is_resumed:
			self.move_group.stop()
			self.new_traj_planned = False

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

	def execute_interruptable_trajectory(self, continuous_traj):
		self.new_traj_planned = True
		interr_robo_traj = moveit_msgs.msg.RobotTrajectory() 	# this is the message type that is published
		interr_traj = JointTrajectory()
		new_waypoint = JointTrajectoryPoint()

		interr_traj.header = continuous_traj.joint_trajectory.header
		interr_traj.joint_names = continuous_traj.joint_trajectory.joint_names
		
		# intiallly add all points to the trajectory
		interr_traj.points = continuous_traj.joint_trajectory.points
		interr_robo_traj.joint_trajectory = interr_traj

		while not rospy.is_shutdown():
			if self.is_resumed:
				if (self._send_traj_to_manipulator(interr_robo_traj)):
					rospy.loginfo("Trajectory execution completed!")
					break
				elif not self.new_traj_planned:
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
							
							self.new_traj_planned = True
							break


if __name__ == '__main__':
	signal.signal(signal.SIGINT, signal_handler)
	signal.signal(signal.SIGTERM, signal_handler)

	ur5 = InterruptableTrajectory()

	# print("Please wait, configuring... \nMoving to start position...")
	# ur5.move_to_joint_state('start_joint_state')

	# # obtaining moveit's trajectory
	# (ur5_traj, fraction) = ur5.get_cartesian_trajectory(ur5.generate_waypoints_0(), 0.01)
	# print("Obtained MoveIt's trajectory that is: ", str(fraction*100), " percent complete.")

	# ur5.execute_interruptable_trajectory(ur5_traj)