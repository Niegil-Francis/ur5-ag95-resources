#!/usr/bin/python

# generic imports
import sys
import time

# ROS imports
import rospy

# MoveIt imports
import moveit_commander
import moveit_msgs.msg

# message imports
from dh_gripper_msgs.msg import GripperState

# vision imports
import cv2
import pyrealsense2 as rs

# math related imports
from math import pi
import numpy as np
from rospy.core import rospyinfo
import tf
from tf.transformations import quaternion_matrix
from tf.transformations import translation_matrix
from scipy.spatial.transform import Rotation as R

# speech recognition imports
import speech_recognition as sr
import pyttsx3

class RecognizeColor:
	"""
	A utility class to recognize color name and obtain the coordinates of
	the centroid of the object's contour. It uses speech recognition and
	keyboard input mutually exclusively.
	"""

	def __init__(self, use_speech_recognition = False):
		"""
		Initialize the tools for recognition
		"""
		self.use_speech_recognition = use_speech_recognition
		if use_speech_recognition:
			self.r = sr.Recognizer()
			self.m = sr.Microphone()

		# TODO: add parameterized constructor

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

		self.low_H = 0
		self.low_S = 0
		self.low_V = 112
		self.high_H = 180
		self.high_S = 74
		self.high_V = 255

	def get_color_from_speech(self):
		"""
		Returns the color value from speech

		1 -> Red | 2 -> Green | 3 -> Yellow
		"""
		color = 0
		if self.use_speech_recognition:
			print("A moment of silence, please...")
			with self.m as source: self.r.adjust_for_ambient_noise(source)
			print("Set minimum energy threshold to {}".format(self.r.energy_threshold))
			while color==0:
				print("Say something!")
				engine = pyttsx3.init()
				engine.say("Please specify the color")
				engine.runAndWait()
				with self.m as source: audio = self.r.listen(source)
				print("Got it! Now to recognize it...")
				try:
					# recognize speech using Google Speech Recognition
					value = self.r.recognize_google(audio)

					# we need some special handling here to correctly print unicode characters to standard output
					if str is bytes:  # this version of Python uses bytes for strings (Python 2)
						print(u"You said {}".format(value).encode("utf-8"))
						if value == "red":
							color = 1
						elif value == "green":
							color = 2           
						elif value == "yellow":
							color = 3          
					else:  # this version of Python uses unicode for strings (Python 3+)
						print("You said {}".format(value))
				except sr.UnknownValueError:
					print("Oops! Didn't catch that")
				except sr.RequestError as e:
					print("Uh oh! Couldn't request results from Google Speech Recognition service; {0}".format(e))
			return color
		else:
			print("Cannot use speech recognition")
			return color
	
	def get_color_from_keyboard(self):
		"""
		Utility method to aid providing color value from keyboard

		1 -> Red | 2 -> Green | 3 -> Yellow
		"""
		if not self.use_speech_recognition:
			return input("Enter the color code: [1] Red | [2] Green | [3] Yellow : ") 
		else:
			print("Cannot use keyboard input")
			return 0

	def get_camera_coordinates(self):
		"""
		Returns the 3D camera coordinate of the centroid of the object
		"""	
		color_code = 0

		if self.use_speech_recognition:
			color_code = self.get_color_from_speech()
		else:
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
		depth_sensor = profile.get_device().first_depth_sensor()
		depth_scale = depth_sensor.get_depth_scale()

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

			# displaying the binary image
			# cv2.imshow('Binary', in_range_mask)
			
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
						# z = depth * depth_scale 

						# obtain the depth pixel at the centroid of the contour
						depth_point = rs.rs2_project_color_pixel_to_depth_pixel(
													depth_frame.get_data(), depth_scale, self.depth_min, self.depth_max,
													depth_intrin, color_intrin, depth_to_color_extrin, color_to_depth_extrin, [px,py])

						# centeroid in camera coordinate
						# TODO: What does this logic do?
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
	the next position to reach.
	"""

	def __init__(self, gripper_offset=200.0, z_safety=200.0):
		"""
			Parameters
			==========

			- gripper_offset: int : Distance of gripper centre point from tool0 link (in mm)
			- z_safety: int : Minimum distance of tool0 from the table (in mm)
		"""
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('moveToObj', anonymous=True)

		self.gripper_pub = rospy.Publisher('gripper_pose_1', GripperState, queue_size=1)
		self.gripper_msg = GripperState()

		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		group_name = "manipulator"
		self.group = moveit_commander.MoveGroupCommander(group_name)

		# self.scene.add_

		print("Pausing (init)...")
		time.sleep(1)
		print("...done!")

		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
												moveit_msgs.msg.DisplayTrajectory,
												queue_size=20)
		
		self.z_safety = z_safety
		self.gripper_offset = gripper_offset

	def get_base_link_coordinates(self, camera_coord):
		"""
		Returns the coordinate of the object's centeroid wrt base_link from the camera coordinate.
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

	def gripper_open(self):
		"""
		Opens the gripper
		"""
		if not rospy.is_shutdown():
			self.gripper_msg.target_position = 1000
			self.gripper_msg.target_force = 20

			self.gripper_pub.publish(self.gripper_msg)
			
			# after sending the open gripper command, sleep for 2 seconds
			print("Pausing (gripper_open)...")
			time.sleep(2)
			print("...done!")
	
	def gripper_close(self):
		"""
		Closes the gripper
		"""
		if not rospy.is_shutdown():
			self.gripper_msg.target_position = 20
			self.gripper_msg.target_force = 20

			self.gripper_pub.publish(self.gripper_msg)
			
			# after sending the close gripper command, sleep for 2 seconds
			print("Pausing (gripper_close)...")
			time.sleep(2)
			print("...done!")

	def get_goal_pose(self, base_link_coord):
		"""
		Converts the base link coordinate location to goal pose.
		A new parameter maybe added to this method to provide orientation information

		Parameters
		=====
		- base_coord : vector describing the goal bose coordinates

		Returns
		---
		Goal pose using input parameters
		"""
		pose_goal = self.group.get_current_pose().pose
		
		pose_goal.position.x = base_link_coord[0][0]
		pose_goal.position.y = base_link_coord[1][0]
		pose_goal.position.z = base_link_coord[2][0] + (self.gripper_offset/1000)

		if pose_goal.position.z < (self.z_safety/1000):
			pose_goal.position.z = (self.z_safety/1000)
			print("Z pose set to Z-safety limit!")
		
		pose_goal.orientation.x = -0.926984315176
		pose_goal.orientation.y = 0.374568844855
		pose_goal.orientation.z = -0.00467163233836
		pose_goal.orientation.w = 0.019401951777

		return pose_goal

	def move_to_location(self, goal_pose):
		"""
		Move the EE to the goal pose

		Parameters
		---
		- goal_pose: pose to which path must be planned
		"""
		current_pose = self.group.get_current_pose().pose
		print("Planning path from pose (current pose):  ")
		print(current_pose)

		print("To pose (goal pose): ")
		print(goal_pose)

		self.group.set_pose_target(goal_pose)

		plan = self.group.plan()
		
		print("Planning the path. Check RViz for visualization. Path will be executed after 5 seconds...")
		rospy.sleep(5)
		print("Executing path...")

		self.group.go(wait=True)
		self.group.stop()
		self.group.clear_pose_targets()

	def get_place_location_pose(self):
		"""
		Returns
		---
		Returns the pose at which the object must to placed.
		"""
		pose_goal = self.group.get_current_pose().pose

		pose_goal.position.x = 0.697925644962
		pose_goal.position.y = 0.102895313812
		pose_goal.position.z = 0.225430132411

		pose_goal.orientation.x = 0.720109681727
		pose_goal.orientation.y = -0.692563162952
		pose_goal.orientation.z = -0.0219631139694
		pose_goal.orientation.w = 0.0362757939835

		return pose_goal

if __name__ == '__main__':
	ur_control = UR5Control(200.0)

	ur_control.gripper_open()

	recognize_color = RecognizeColor()

	camera_coord  = recognize_color.get_camera_coordinates()

	print("Camera coordinates: ", camera_coord)

	base_link_coord = ur_control.get_base_link_coordinates(camera_coord)
	
	print("Base link coordinates: ", base_link_coord)

	initial_pose = ur_control.group.get_current_pose().pose

	ur_control.move_to_location(ur_control.get_goal_pose(base_link_coord))

	ur_control.gripper_close()
	
	ur_control.move_to_location(ur_control.get_place_location_pose())

	ur_control.gripper_open()
	
	ur_control.move_to_location(initial_pose)
	
	print("Done!")
