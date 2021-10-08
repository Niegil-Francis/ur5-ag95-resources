#!/usr/bin/python3

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
from imutils import paths

# speech recognition imports
import speech_recognition as sr
import pyttsx3
import threading
import imutils

# Python code to illustrate Sending mail with attachments
# from your Gmail account 
  
# libraries to be imported
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email import encoders

class Demo:
	"""
	A utility class to take pictures and a video of guests
	"""

	def __init__(self, take_video = False):
		"""
		Initialize the video writer and set camera properties
		"""
		self.take_video = take_video
		if take_video:
			self.frame_width = int(640)
			self.frame_height = int(480)
			self.size = (self.frame_width, self.frame_height)
			self.writer = cv2.VideoWriter('filename.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, self.size)


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
		self.counter_name=0

		# Initializing speech 
		self.r = sr.Recognizer()
		self.m = sr.Microphone()

	def store_camera_frames(self):
		"""
		Stores the camera frames at a constant rate and saves a video
		"""	
		
		# creating a pipeline to get depth and color images
		pipeline = rs.pipeline()
		config = rs.config()
		config.enable_stream(rs.stream.depth, self.depth_resolution_x, self.depth_resolution_y, rs.format.z16, self.depth_fps) 
		config.enable_stream(rs.stream.color, self.color_resolution_x, self.color_resolution_y, rs.format.bgr8, self.color_fps)
		profile = pipeline.start(config)
		depth_sensor = profile.get_device().first_depth_sensor()
		depth_scale = depth_sensor.get_depth_scale()

		camera_coord = []
		flag=100
		while True:
			# wait for a new frame and then get the depth and color frame 
			frames = pipeline.wait_for_frames() 
			depth_frame = frames.get_depth_frame()
			color_frame = frames.get_color_frame()
			if not depth_frame or not color_frame:
				continue
			elif flag==0:
				break
			flag-=1
		
		# temp var to store camera coordinate; this coordinate might be discarded by the logic in this code that follows
		temp_camera_coord = []

		# create numpy array of depth and color frames
		depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())
		cv2.imwrite("src/ur5-ag95-resources/cobot-control/scripts/demo_guest/data/input/colour"+str(self.counter_name)+".jpg", color_image)
		
		self.counter_name+=1
		cv2.destroyAllWindows()		# after pressing any key close all the window

	def stitching(self):
		print("[INFO] loading images...")
		imagePaths = sorted(list(paths.list_images(args.images)))
		images = []
		# loop over the image paths, load each one, and add them to our
		# images to stich list
		for imagePath in imagePaths:
			image = cv2.imread(imagePath)
			images.append(image)
		# initialize OpenCV's image sticher object and then perform the image
		# stitching
		print("[INFO] stitching images...")
		stitcher = cv2.createStitcher() if imutils.is_cv3() else cv2.Stitcher_create()
		(status, stitched) = stitcher.stitch(images)
		# if the status is '0', then OpenCV successfully performed image
		# stitching
		if status == 0:
			# check to see if we supposed to crop out the largest rectangular
			# region from the stitched image
			if args.crop > 0:
				# create a 10 pixel border surrounding the stitched image
				print("[INFO] cropping...")
				stitched = cv2.copyMakeBorder(stitched, 10, 10, 10, 10,
					cv2.BORDER_CONSTANT, (0, 0, 0))
				# convert the stitched image to grayscale and threshold it
				# such that all pixels greater than zero are set to 255
				# (foreground) while all others remain 0 (background)
				gray = cv2.cvtColor(stitched, cv2.COLOR_BGR2GRAY)
				thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]
				# find all external contours in the threshold image then find
				# the *largest* contour which will be the contour/outline of
				# the stitched image
				cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
					cv2.CHAIN_APPROX_SIMPLE)
				cnts = imutils.grab_contours(cnts)
				c = max(cnts, key=cv2.contourArea)
				# allocate memory for the mask which will contain the
				# rectangular bounding box of the stitched image region
				mask = np.zeros(thresh.shape, dtype="uint8")
				(x, y, w, h) = cv2.boundingRect(c)
				cv2.rectangle(mask, (x, y), (x + w, y + h), 255, -1)
				# create two copies of the mask: one to serve as our actual
				# minimum rectangular region and another to serve as a counter
				# for how many pixels need to be removed to form the minimum
				# rectangular region
				minRect = mask.copy()
				sub = mask.copy()
				# keep looping until there are no non-zero pixels left in the
				# subtracted image
				while cv2.countNonZero(sub) > 0:
					# erode the minimum rectangular mask and then subtract
					# the thresholded image from the minimum rectangular mask
					# so we can count if there are any non-zero pixels left
					minRect = cv2.erode(minRect, None)
					sub = cv2.subtract(minRect, thresh)
				# find contours in the minimum rectangular mask and then
				# extract the bounding box (x, y)-coordinates
				cnts = cv2.findContours(minRect.copy(), cv2.RETR_EXTERNAL,
					cv2.CHAIN_APPROX_SIMPLE)
				cnts = imutils.grab_contours(cnts)
				c = max(cnts, key=cv2.contourArea)
				(x, y, w, h) = cv2.boundingRect(c)
				# use the bounding box coordinates to extract the our final
				# stitched image
				stitched = stitched[y:y + h, x:x + w]
			# write the output stitched image to disk
			cv2.imwrite(args.output, stitched)
			cv2.namedWindow("Stitched", cv2.WINDOW_AUTOSIZE)    # Create window with freedom of dimensions
			stitched = cv2.resize(stitched, (1500, 800)) 
			# display the output stitched image to our screen
			cv2.imshow("Stitched", stitched)
			cv2.waitKey(0)
		# otherwise the stitching failed, likely due to not enough keypoints)
		# being detected
		else:
			print("[INFO] image stitching failed ({})".format(status))
			
			
			print("Done!")

	def send_mail(self):   
		fromaddr = "roboticsinnovationslab@gmail.com"
		toaddr = "rilstudents@iisc.ac.in"
		# rilstudents@iisc.ac.in
		
		# instance of MIMEMultipart
		msg = MIMEMultipart()
		
		# storing the senders email address  
		msg['From'] = fromaddr
		
		# storing the receivers email address 
		msg['To'] = toaddr
		
		# storing the subject 
		msg['Subject'] = "RIL Guest Photo"
		
		# string to store the body of the mail
		body = "Automated guest photo email"
		
		# attach the body with the msg instance
		msg.attach(MIMEText(body, 'plain'))
		
		# open the file to be sent 
		filename = args.output
		attachment = open(filename, "rb")
		
		# instance of MIMEBase and named as p
		p = MIMEBase('application', 'octet-stream')
		
		# To change the payload into encoded form
		p.set_payload((attachment).read())
		
		# encode into base64
		encoders.encode_base64(p)
		
		p.add_header('Content-Disposition', "attachment; filename= %s" % filename)
		
		# attach the instance 'p' to instance 'msg'
		msg.attach(p)
		
		# creates SMTP session
		s = smtplib.SMTP('smtp.gmail.com', 587)
		
		# start TLS for security
		s.starttls()
		
		# Authentication
		s.login(fromaddr, "RIL@cpdmiisc2020")
		
		# Converts the Multipart msg into a string
		text = msg.as_string()
		
		# sending the mail
		s.sendmail(fromaddr, toaddr.split(","), text)
		
		# terminating the session
		s.quit()
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

	def move_to_joint_goal(self,poses):
		for i in poses:
			joint_goal = self.group.get_current_joint_values()
			joint_goal[0] = i[0]
			joint_goal[1] = i[1]
			joint_goal[2] = i[2]
			joint_goal[3] = i[3]
			joint_goal[4] = i[4]
			joint_goal[5] = i[5]

			# The go command can be called with joint values, poses, or without any
			# parameters if you have already set the pose or joint target for the group
			self.group.go(joint_goal, wait=True)

			# Calling ``stop()`` ensures that there is no residual movement
			self.group.stop()


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

class args:
	images = "src/ur5-ag95-resources/cobot-control/scripts/demo_guest/data/input/"
	crop = 1
	output = "src/ur5-ag95-resources/cobot-control/scripts/demo_guest/data/output/stitched.jpg"

if __name__ == '__main__':
	ur_control = UR5Control(200.0)

	# ur_control.gripper_open()

	demo = Demo()
	home = [[0.0, -1.5708439985858362, 5.992112710373476e-05, -1.5708444754229944, -3.5587941304981996e-05, 7.190534961409867e-05]]
	ur_control.move_to_joint_goal(home)
	poses=[[-0.2770283857928675, -1.1574118773089808, -2.130111042653219, 0.10499870777130127, 1.6243500709533691, 0.023737754672765732],[-0.6771419684039515, -1.1574233214007776, -2.1301472822772425, 0.10503458976745605, 1.624266266822815, 0.023713786154985428],[-1.0323670546161097, -1.1573994795428675, -2.130134884511129, 0.10503458976745605, 1.6242902278900146, 0.023749738931655884],[-1.355570141469137, -1.1574233214007776, -2.1301711241351526, 0.10503458976745605, 1.6243021488189697, 0.023761723190546036],[-1.6579378286944788, -1.157435719166891, -2.130099121724264, 0.10501062870025635, 1.6243141889572144, 0.02372577041387558],[-1.961966339741842, -1.157435719166891, -2.130134884511129, 0.10501062870025635, 1.6243021488189697, 0.023713786154985428]]
	for count,i in enumerate(poses):
		ur_control.move_to_joint_goal([i])
		if count==0:
			engine = pyttsx3.init()
			engine.say('Get....ready.....in....')
			engine.runAndWait()
			for i in ['Three','Two','One']:
				engine.say(i)
				engine.runAndWait()
				time.sleep(1)
			engine.say("Smile...")
			engine.runAndWait()
		demo.store_camera_frames()
		if count==0:
			engine.say("Do not move yet")
			engine.runAndWait()

	engine.say("The picture is taken and getting stitched")
	engine.runAndWait()
	ur_control.move_to_joint_goal(home)

	demo.stitching()

	engine.say("Sending the automated mail")
	engine.runAndWait()
	demo.send_mail()




