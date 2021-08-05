#!/usr/bin/python3

# generic imports
import sys
import time
import signal

# ROS imports
import rospy
from rospkg import RosPack

# MoveIt imports
import moveit_commander
import moveit_msgs.msg

# message imports
from dh_gripper_msgs.msg import GripperState
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# vision imports
import cv2
import pyrealsense2 as rs

# math related imports
import numpy as np
import tf
from tf.transformations import quaternion_matrix
from tf.transformations import translation_matrix
from scipy.spatial.transform import Rotation as R

# misc imports
import yaml

class RecognizeColor:
	"""
	A utility class to recognize color name and obtain the coordinates of
	the centroid of the object's contour.
	"""

	def __init__(self, color_image_topic, depth_image_topic, depth_image_info_topic):
		"""
		Subscribe to the required topics

		Parameters
		----------

		* color_image_topic: Topic name on which color image is published
		* depth_image_topic: Topic name on which depth image is published
		* depth_image_info_topic: Topic name on which depth image info is published
		"""
		self.bridge = CvBridge()
		self.color_image_sub = rospy.Subscriber(color_image_topic, Image, self.colorCallback)
		self.depth_image_sub = rospy.Subscriber(depth_image_topic, Image, self.depthCallback)
		self.depth_image_info_sub = rospy.Subscriber(depth_image_info_topic, CameraInfo, self.depthInfoCallback)
		self.color_image = None
		self.depth_image = None

		# intrinsic parameters required to get 3D coordinates
		self.intrinsics = None

		self.depth_max = 1.0	# metres

		# default values
		self.low_H = 0
		self.low_S = 0
		self.low_V = 112
		self.high_H = 180
		self.high_S = 74
		self.high_V = 255
		
	def depthCallback(self, data):
		try:
			self.depth_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
		except CvBridgeError as e:
			rospy.logerr(e)
	
	def colorCallback(self,data):
		try:
			self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)
		
	def depthInfoCallback(self, depth_info):
		"""
		Sets the instrinsics value from the camera info topic for depth
		"""
		try:
			if self.intrinsics:
				return
			self.intrinsics = rs.intrinsics()
			self.intrinsics.width = depth_info.width
			self.intrinsics.height = depth_info.height
			self.intrinsics.ppx = depth_info.K[2]
			self.intrinsics.ppy = depth_info.K[5]
			self.intrinsics.fx = depth_info.K[0]
			self.intrinsics.fy = depth_info.K[4]
			if depth_info.distortion_model == 'plumb_bob':
				self.intrinsics.model = rs.distortion.brown_conrady
			elif depth_info.distortion_model == 'equidistant':
				self.intrinsics.model = rs.distortion.kannala_brandt4
			self.intrinsics.coeffs = [i for i in depth_info.D]
		except CvBridgeError as e:
			rospy.logerr(e)
			return

	def getColorFromKeyboard(self):
		"""
		Utility method to aid providing color value from keyboard


		1 -> Red | 2 -> Green | 3 -> Yellow
		"""
		return input("Enter the color code: [1] Red | [2] Green | [3] Yellow : ") 

	def getCameraCoordinates(self):
		"""
		Returns
		-------
		The 3D camera coordinate of the centroid of the object or None
		"""	
		color_code = 0
		color_code = self.getColorFromKeyboard()

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

		if (self.color_image is not None) and (self.depth_image is not None):
			camera_coord = []
		
			while True:
				# temp var to store camera coordinate; this coordinate might be discarded by the logic in this code that follows
				temp_camera_coord = []

				# displaying the image
				cv2.imshow('Color image', self.color_image)

				# check is the requested color is present in the image
				in_range_mask = cv2.inRange(self.color_image, (self.low_H, self.low_S, self.low_V), (self.high_H, self.high_S, self.high_V))

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
							cv2.drawContours(self.color_image, [c], -1, (0, 255, 0), 2)
							depth = self.depth_image[py, px].astype(float)

							# obtaining the 3D coordinates by deprojecting
							temp_camera_coord = rs.rs2_deproject_pixel_to_point(self.intrinsics, list((float(px),float(py))), depth)
							temp_camera_coord = np.reshape(temp_camera_coord,(-1,1))
							temp_camera_coord = np.concatenate([temp_camera_coord,np.reshape([1],(-1,1))], axis=0)
						else:
							px, py = 0, 0
					
					# displaying the color image with contours
					cv2.imshow('Contours', self.color_image)  
			
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
		else:
			return None

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
		signal.signal(signal.SIGINT, self.exit_gracefully)
		signal.signal(signal.SIGTERM, self.exit_gracefully)
		
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('advPickPlace', anonymous=True)

		self.gripper_pub = rospy.Publisher('gripper_pose_1', GripperState, queue_size=1)
		self.gripper_msg = GripperState()

		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		group_name = "manipulator"
		self.group = moveit_commander.MoveGroupCommander(group_name)
		self.group.set_planning_time(10)
		self.group.set_planner_id('RRT')

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
		
	def getBaseLinkCoordinates(self, camera_coord):
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

	def controlGripper(self, target_force, target_position):
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
			print("Pausing (controlGripper)...")
			time.sleep(2)
			print("...done!")

	def openGripper(self, target_force=5):
		"""
		Utility function that opens the gripper

		Parameters
		---------
		target_force: float:
			Force applied on the fingers
		"""
		self.controlGripper(target_force=target_force, target_position=1000)
	
	def closeGripper(self, target_force=5):
		"""
		Utility function that closes the gripper

		Parameters
		---------
		target_force: float:
			Force applied on the fingers
		"""
		self.controlGripper(target_force=target_force, target_position=20)
	
	def getPickGoalPose(self, base_link_coord):
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
		pose_goal = self.group.get_current_pose().pose
		
		pose_goal.position.x = base_link_coord[0][0]
		pose_goal.position.y = base_link_coord[1][0]
		pose_goal.position.z = base_link_coord[2][0]

		pose_goal.orientation.x = self.determined_poses['pickOrientation']['orientation']['x']
		pose_goal.orientation.y = self.determined_poses['pickOrientation']['orientation']['y']
		pose_goal.orientation.z = self.determined_poses['pickOrientation']['orientation']['z']
		pose_goal.orientation.w = self.determined_poses['pickOrientation']['orientation']['w']

		return pose_goal

	def moveToLocation(self, goal_pose):
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
		self.group.clear_pose_targets()
		current_pose = self.group.get_current_pose().pose
		print("Planning path from pose (current pose):  ")
		print(current_pose)

		if goal_pose.position.z < self.z_safety_limit:
			goal_pose.position.z = self.z_safety_limit
			rospy.logwarn("The goal pose has invalid Z coordinate for TCP. Coordinate overridden to Z = 0.02")

		print("To pose (goal pose): ")
		print(goal_pose)

		self.group.set_pose_target(goal_pose)

		plan = self.group.plan()
		
		print("Planning the path. Check RViz for visualization.")
		user_in = input('Do you want to execute the path on the real cobot? [y|n] ')

		if user_in == 'y':
			print("Executing path after 3 seconds...")
			rospy.sleep(3)
			self.group.go(wait=True)
			self.group.stop()
			self.group.clear_pose_targets()
			return 1
		else:
			print("Path no executed!")
			self.group.clear_pose_targets()
			return 0

	def getPoseFromYaml(self, pose_id_name):
		"""
		Returns
		---
		Pose from the YAML file
		"""
		pose_yaml = self.group.get_current_pose().pose

		pose_yaml.position.x = self.determined_poses[pose_id_name]['position']['x']
		pose_yaml.position.y = self.determined_poses[pose_id_name]['position']['y']
		pose_yaml.position.z = self.determined_poses[pose_id_name]['position']['z']

		pose_yaml.orientation.x = self.determined_poses[pose_id_name]['orientation']['x']
		pose_yaml.orientation.y = self.determined_poses[pose_id_name]['orientation']['y']
		pose_yaml.orientation.z = self.determined_poses[pose_id_name]['orientation']['z']
		pose_yaml.orientation.w = self.determined_poses[pose_id_name]['orientation']['w']

		return pose_yaml

	def exit_gracefully(self, *args):
		self.group.stop()
		sys.exit(0)

if __name__ == '__main__':
	color_image_topic = "/camera/color/image_raw"
	depth_image_topic = "/camera/aligned_depth_to_color/image_raw"
	depth_image_info_topic = "/camera/aligned_depth_to_color/camera_info"

	ur_control = UR5Control(0.02)

	ur_control.moveToLocation(ur_control.getPoseFromYaml('startPose'))

	recognize_color = RecognizeColor(color_image_topic, depth_image_topic, depth_image_info_topic)

	print("Waiting for a second to receive data...")
	time.sleep(1)

	camera_coord  = recognize_color.getCameraCoordinates()

	print("Camera coordinates: ", camera_coord)

	if camera_coord is not None:
		base_link_coord = ur_control.getBaseLinkCoordinates(camera_coord)
		print("Base link coordinates: ", base_link_coord)
	else:
		rospy.logerr("Could not connect to camera!")
	
	initial_pose = ur_control.group.get_current_pose().pose

	ur_control.moveToLocation(ur_control.getPickGoalPose(base_link_coord))

	ur_control.closeGripper()
	
	ur_control.moveToLocation(ur_control.getPoseFromYaml('placePose'))

	ur_control.openGripper()
	
	ur_control.moveToLocation(initial_pose)
	
	print("Done!")
