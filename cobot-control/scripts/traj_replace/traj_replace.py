#!/usr/bin/python3

# Adding the common python files to path
import sys
from rospkg import RosPack
sys.path.insert(1, RosPack().get_path('cobot-control') + "/scripts/common/")
from ur5control import UR5Control

import signal
import copy

# ROS imports
import rospy

# message imports
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int8

# misc imports
import yaml

def signal_handler(*args):
	sys.exit()
	
class TrajectoryReplace(UR5Control):

	def __init__(self, is_sim=True, planning_time=10, planner_id='RRT', z_safety_limit=0.01):
		super().__init__(planning_time=planning_time, planner_id=planner_id, z_safety_limit=z_safety_limit)
		rospy.init_node('traj_replace', anonymous=True)

		dir_path = RosPack().get_path('cobot-control') + "/scripts/traj_replace/"
		
		# opening YAML file for topics
		with open(dir_path + "topics.yaml", 'r') as stream:
			try:
				self.topics = yaml.safe_load(stream)
			except yaml.YAMLError as exc:
				print(exc)
				print("Exiting because some issues with loading YAML")
				sys.exit(0)

		# CONSTANTS: Trajectory Type
		self.TT_STOP = 0
		self.TT_HOME = 1
		self.TT_START_POSE = 2
		self.TT_REPLACE_TRAJ = 3
		self.TT_SCALE_VEL = 4

		# initializing variables
		self.traj_type = self.TT_STOP
		self.traj_msg = JointTrajectory()

		# publishers
		if is_sim:
			self.traj_pub = rospy.Publisher(self.topics['simulation_control'], JointTrajectory, queue_size=1)
		else:
			self.traj_pub = rospy.Publisher(self.topics['hardware_control'], JointTrajectory, queue_size=1)

		# subscribers
		self.traj_type_sub = rospy.Subscriber('traj_type', Int8, self.traj_type_callback)

		self.generate_traj_0()

		self.traj_pub.publish(self.traj_msg)

	def traj_type_callback(self, data):
		self.traj_type = data.data

	def generate_traj_0(self):
		"""
			Populates the trajectory message with a default pre-defined 
			trajectory. Should be called in the constructor
		"""
		waypoints = []

		wpose = self.move_group.get_current_pose().pose
		wpose.position.z +=  0.1 
		wpose.position.y +=  0.2
		waypoints.append(copy.deepcopy(wpose))

		wpose.position.x +=  0.2
		waypoints.append(copy.deepcopy(wpose))

		(traj_0, _) = self.get_cartesian_trajectory(waypoints, 0.01)

		self.traj_msg.header = traj_0.header
		self.traj_msg.joint_names = traj_0.joint_names

		waypoint_num = len(traj_0.points)

		for i in range(0, waypoint_num):
			point = JointTrajectoryPoint()
			point.positions.append(traj_0.points[i].positions)
			point.time_from_start = rospy.Duration(0.6)
			self.traj_msg.points.append(point)
			pass

	# def _send_traj_to_manipulator(self, robo_traj):
	# 	"""
	# 	Sends the trajectory execution command to the manipulator and starts the timer.

	# 	Parameters
	# 	----------
	# 	robo_traj : RobotTrajectory: The trajectory to be executed

	# 	Returns
	# 	-------
	# 	True if the trajectory was executed without any interruption, else false.
	# 	"""
	# 	self.execution_start_time = rospy.get_rostime()
	# 	return self.move_group.execute(robo_traj, wait=True)

	# def execute_cartesian_traj_with_rand_wp(self):
	# 	"""
	# 	Searches for a random trajectory and executes it.
	# 	Returns
	# 	-------
	# 	True if the trajectory was executed without any interruption, else false.
	# 	"""
		
	# 	# obtaining moveit's trajectory
	# 	(continuous_traj, fraction) = self.get_cartesian_trajectory(self.generate_waypoints_0(), 0.01)
	# 	while(fraction < 0.3):
	# 		(continuous_traj, fraction) = self.get_cartesian_trajectory(self.generate_waypoints_0(), 0.01)
	# 	print("Obtained MoveIt's trajectory that is: ", str(fraction*100), " percent complete.")
	# 	self.interr_traj.header = continuous_traj.joint_trajectory.header
	# 	self.interr_traj.joint_names = continuous_traj.joint_trajectory.joint_names

	# 	# intiallly add all points to the trajectory
	# 	self.interr_traj.points = continuous_traj.joint_trajectory.points
	# 	self.interr_robo_traj.joint_trajectory = self.interr_traj
	# 	self.new_traj_planned = True
	# 	return self._send_traj_to_manipulator(self.interr_robo_traj)

if __name__ == '__main__':
	signal.signal(signal.SIGINT, signal_handler)

	ur5 = TrajectoryReplace(is_sim=True)