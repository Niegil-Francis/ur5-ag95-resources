#!/usr/bin/python3

# Adding the common python files to path
import sys
from rospkg import RosPack
sys.path.insert(1, RosPack().get_path('cobot-control') + "/scripts/common/")
from ur5control import UR5Control

import copy

# ROS imports
import rospy

# message imports
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int8

# misc imports
import yaml

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
		self.positions = []
		self.velocities = []
		self.accelerations = []
		self.time_from_start = []

		# publishers
		self.is_sim = is_sim
		if is_sim:
			self.traj_pub = rospy.Publisher(self.topics['simulation_control'], JointTrajectory, queue_size=1)
		else:
			self.traj_pub = rospy.Publisher(self.topics['hardware_control'], JointTrajectory, queue_size=1)

		# subscribers
		self.traj_type_sub = rospy.Subscriber('traj_type', Int8, self.traj_type_callback)

		self.generate_traj_0()

	def traj_type_callback(self, data):
		self.traj_type = data.data

	def execute_trajectory(self):
		"""
		Executes the trajectory one position after another
		"""
		num_points = len(self.positions)
		new_point = JointTrajectoryPoint()

		loop_rate = rospy.Rate(125)

		k = 0 					# iteration variable
		joint_tolerance = 0.1	# set-point tolerance at which next set-point is sent
		if self.is_sim:
			joint_tolerance = 0.001

		while not rospy.is_shutdown() and k != num_points-1:
			# check if the current position is reached or not
			if not self.is_within_tolerance(list(self.positions[k]), 
											self.move_group.get_current_joint_values(),
											joint_tolerance):
				# for some reason, we need to keep on publishing this
				# self.traj_msg.header.stamp = rospy.Time.now()
				self.traj_pub.publish(self.traj_msg)
			else:
				k = k + 1			# publish the next set-point position
				self.traj_msg.points.clear()

				new_point.positions = self.positions[k]
				new_point.velocities = self.velocities[k]
				new_point.accelerations = self.accelerations[k]
				if self.is_sim:
					new_point.time_from_start = self.time_from_start[k] - self.time_from_start[k-1]
				else:
					new_point.time_from_start = self.time_from_start[k] - self.time_from_start[k-1] + rospy.Duration(0.25)
				self.traj_msg.points.append(new_point)
				
				self.traj_msg.header.stamp = rospy.Time.now()
				self.traj_pub.publish(self.traj_msg)
			loop_rate.sleep()

		self.positions.clear()
		self.time_from_start.clear()

	def generate_traj_0(self):
		"""
		Obtain the trajectory positions with a default pre-defined 
		trajectory. Should be called in the constructor.
		"""
		waypoints = []

		wpose = self.move_group.get_current_pose().pose

		wpose.position.z +=  0.3
		waypoints.append(copy.deepcopy(wpose))
		wpose.position.y +=  0.2
		waypoints.append(copy.deepcopy(wpose))
		wpose.position.x +=  0.1  
		waypoints.append(copy.deepcopy(wpose))
		
		wpose.position.z -=  0.3
		waypoints.append(copy.deepcopy(wpose))
		wpose.position.y -=  0.2
		waypoints.append(copy.deepcopy(wpose))
		wpose.position.x -=  0.1  
		waypoints.append(copy.deepcopy(wpose))

		(traj_0, fraction) = self.get_cartesian_trajectory(waypoints, 0.01)

		rospy.loginfo("Fraction of trajectory planned: " + str(fraction))

		self.traj_msg.joint_names = traj_0.joint_trajectory.joint_names

		waypoint_num = len(traj_0.joint_trajectory.points)

		for i in range(0, waypoint_num):
			self.positions.append(traj_0.joint_trajectory.points[i].positions)
			self.time_from_start.append(traj_0.joint_trajectory.points[i].time_from_start)
			self.velocities.append(traj_0.joint_trajectory.points[i].velocities)
			self.accelerations.append(traj_0.joint_trajectory.points[i].accelerations)

if __name__ == '__main__':
	ur5 = TrajectoryReplace(is_sim=True)

	ur5.execute_trajectory()

	rospy.loginfo("Press CTRL+C to exit node")
	rospy.spin()