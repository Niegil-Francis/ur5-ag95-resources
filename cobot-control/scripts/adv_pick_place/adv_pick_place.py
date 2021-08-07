#!/usr/bin/python3

# generic imports
import sys
import time
import signal
import geometry_msgs

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

# vision imports
import cv2
import pyrealsense2 as rs

# math related imports
import numpy as np
import tf
from tf.transformations import quaternion_matrix
from tf.transformations import translation_matrix
from math import dist, fabs, cos

# misc imports
import yaml

class RecognizeColor:
	"""
	A utility class to recognize color name and obtain the coordinates of
	the centroid of the object's contour.
	"""

	def __init__(self):
		"""
		Initializes the default values for the camera.
		"""
		# default values
		self.low_H = 0
		self.low_S = 0
		self.low_V = 112
		self.high_H = 180
		self.high_S = 74
		self.high_V = 255

		# setting the camera limits
		self.depth_resolution_x = 640
		self.depth_resolution_y = 480
		self.color_resolution_x = 640
		self.color_resolution_y = 480
		self.depth_fps = 30
		self.color_fps = 60

		# minimum and maximum depth to perceive
		self.depth_min = 0.11
		self.depth_max = 1.0
		
	def get_color_from_keyboard(self):
		"""
		Utility method to aid providing color value from keyboard


		1 -> Red | 2 -> Green | 3 -> Yellow
		"""
		return input("Enter the color code: [1] Red | [2] Green | [3] Yellow : ") 

	def get_camera_coordinates(self):
		"""
		Returns
		-------
		The 3D camera coordinate of the centroid of the object or None
		"""	
		color_code = 0
		color_code = self.get_color_from_keyboard()

		if color_code == 1:
			self.low_H = 0
			self.low_S = 0
			self.low_V = 112
			self.high_H = 180
			self.high_S = 74
			self.high_V = 255
		elif color_code == 2:
			self.low_H = 0
			self.low_S = 81
			self.low_V = 0
			self.high_H = 180
			self.high_S = 152
			self.high_V = 51
		
		# creating a pipeline to get depth and color images
		pipeline = rs.pipeline()
		config = rs.config()
		config.enable_stream(rs.stream.depth, self.depth_resolution_x, self.depth_resolution_y, rs.format.z16, self.depth_fps) 
		config.enable_stream(rs.stream.color, self.color_resolution_x, self.color_resolution_y, rs.format.bgr8, self.color_fps)
		profile = pipeline.start(config)
		depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

		camera_coord = []

		while True:
			# wait for a new frame and then get the depth and color frame 
			frames = pipeline.wait_for_frames() 
			depth_frame = frames.get_depth_frame()
			color_frame = frames.get_color_frame()
			if not depth_frame or not color_frame:
				continue

			# temp var to store camera coordinate; this coordinate might be discarded by the logic in this code that follows
			temp_camera_coord = []

			# create numpy array of depth and color frames
			depth_image = np.asanyarray(depth_frame.get_data())
			color_image = np.asanyarray(color_frame.get_data())

			# cv2.imwrite("colour.jpg", color_image)
			depth_image = depth_image		# TODO: What does this do?

			cv2.imshow('org', color_image)  						# displaying the image

			# check is the requested color is present in the image
			in_range_mask = cv2.inRange(color_image, (self.low_H, self.low_S, self.low_V), (self.high_H, self.high_S, self.high_V))

			# obtain a contour of the mask
			contours, _ = cv2.findContours(in_range_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_L1)

			# loop through all contours
			for c in contours:
				if cv2.contourArea(c) < 300:  # ignore the contour area when it is less than 
					continue
				elif cv2.contourArea(c) > 7000:  # ignore the contour area when it is greater than
					continue
				else:
					M = cv2.moments(c)  	# image moment of the contour
					if M["m00"] != 0:
						px = int(M["m10"] / M["m00"])  # x coordinate centroid
						py = int(M["m01"] / M["m00"])  # y coordinate centroid
						cv2.drawContours(color_image, [c], -1, (0, 255, 0), 2)
						depth_intrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
						color_intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

						depth_to_color_extrin =  profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to( profile.get_stream(rs.stream.color))
						color_to_depth_extrin =  profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to( profile.get_stream(rs.stream.depth))

						depth = depth_image[py, px].astype(float)

						# obtain the depth pixel at the centroid of the contour
						depth_point = rs.rs2_project_color_pixel_to_depth_pixel(
													depth_frame.get_data(), depth_scale, self.depth_min, self.depth_max,
													depth_intrin, color_intrin, depth_to_color_extrin, color_to_depth_extrin, [px,py])

						# centeroid in camera frame; conversion to homogeneous coordinate
						temp_camera_coord = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_point, depth)
						temp_camera_coord = np.reshape(temp_camera_coord,(-1,1))
						temp_camera_coord = np.concatenate([temp_camera_coord,np.reshape([1],(-1,1))], axis=0)
					else:
						px, py = 0, 0
					
					cv2.imshow('Contours', color_image)  # displaying the color image with contours
			
			# to eliminate the cases when no camera coordinate is obtained		
			if len(temp_camera_coord)!=0:
				# to eliminate the cases when depth estimated is more than the allowable range
				if temp_camera_coord[3][0] >= self.depth_max * 1000:	# depth_max parameter is in meters while coordinates are in mm
					cv2.waitKey(1)
					continue
				# eliminating condition that camera is touching the table
				if temp_camera_coord[3][0] <= 0:
					cv2.waitKey(1)
					continue
			else:
				cv2.waitKey(1)
				continue
			print("Press 'q' to perform next step")

			if cv2.waitKey(30)& 0xFF == ord('q'):
				camera_coord = temp_camera_coord
				break

		cv2.destroyAllWindows()		# after pressing any key close all the window
		
		# the camera coordinates are obtained in mm while the transformation matrix values
		# are obtained in m. Hence, the coordinates are converted from mm to m while
		# preserving the last element in the vector.
		camera_coord = camera_coord/1000
		camera_coord[3][0] = 1

		return camera_coord

class UR5Control:
	"""
	A utility class containing methods that aid in moving the UR5 arm
	as well as the AG95 gripper. It also contains methods to estimate
	the next position to reach. It reads pose values from YAML file.
	"""

	def __init__(self, z_safety_limit = 0.01):
		"""
		Initializes moveit_commander, ros-node, publishers and subscribers.

		Parameters
		----------
		z_safety_limit: float:
			Minimum z coordinate of TCP wrt base_link frame (in m)
		"""
		signal.signal(signal.SIGINT, self.exit_node)
		signal.signal(signal.SIGTERM, self.exit_node)
		
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('advPickPlace', anonymous=True)

		self.gripper_pub = rospy.Publisher('gripper_pose_1', GripperState, queue_size=1)
		self.gripper_msg = GripperState()

		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		group_name = "manipulator"
		self.move_group = moveit_commander.MoveGroupCommander(group_name)
		self.move_group.set_planning_time(10)
		self.move_group.set_planner_id('RRT')

		print("Pausing (init)...")
		time.sleep(1)
		print("...done!")

		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
												moveit_msgs.msg.DisplayTrajectory,
												queue_size=20)
		self.z_safety_limit = z_safety_limit

		# initializing variables
		rp = RosPack()
		dir_path = rp.get_path('cobot-control') + "/scripts/adv_pick_place/"

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
		@param: goal       A list of floats, a Pose or a PoseStamped
		@param: actual     A list of floats, a Pose or a PoseStamped
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
			rospy.logwarn("The goal pose has invalid Z coordinate for TCP. Coordinate overridden to Z = 0.02")

		print("To pose (goal pose): ")
		print(goal_pose)

		self.move_group.set_pose_target(goal_pose)

		plan = self.move_group.plan()
		
		print("Planning the path. Check RViz for visualization.")
		user_in = input('Do you want to execute the path on the real cobot? [y|n] ')

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
		user_in = input('Do you want to execute the path on the real cobot? [y|n] ')

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

	def exit_node(self, *args):
		self.move_group.stop()
		sys.exit(0)

if __name__ == '__main__':
	ur_control = UR5Control()

	ur_control.move_to_joint_state('start_joint_state')

	recognize_color = RecognizeColor()

	print("Waiting for a second to receive data...")
	time.sleep(1)

	camera_coord  = recognize_color.get_camera_coordinates()

	print("Camera coordinates: ", camera_coord)

	if camera_coord is not None:
		base_link_coord = ur_control.get_base_link_coordinates(camera_coord)
		print("Base link coordinates: ", base_link_coord)
	else:
		rospy.logerr("Could not connect to camera!")
	
	initial_pose = ur_control.move_group.get_current_pose().pose

	ur_control.move_to_location(ur_control.get_pick_goal_pose(base_link_coord))

	ur_control.close_gripper()
	
	ur_control.move_to_location(ur_control.get_pose_from_yaml('place_pose'))

	ur_control.open_gripper()
	
	ur_control.move_to_location(initial_pose)
	
	print("Done! Press CTRL+C to exit")
	rospy.spin()
