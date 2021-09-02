#!/usr/bin/python3

# ROS imports
import rospy

# Message imports
import geometry_msgs.msg

# Math imports
from scipy.spatial.transform import Rotation as R

# misc imports
import getch

class UserPoseInput:
	def __init__(self):
		self.user_pose_pub = rospy.Publisher('/user_pose', geometry_msgs.msg.Pose, queue_size=1)

		self.user_pose_msg = geometry_msgs.msg.Pose()
		self.user_pose_msg.position.x = 0.6	# CAUTION!!! DONOT CHANGE THIS VALUE IF YOU DONOT KNOW WHAT U ARE DOING
		self.user_pose_msg.position.y = 0.8 # CAUTION!!! DONOT CHANGE THIS VALUE IF YOU DONOT KNOW WHAT U ARE DOING
		self.user_pose_msg.position.z = 0   # CAUTION!!! DONOT CHANGE THIS VALUE IF YOU DONOT KNOW WHAT U ARE DOING

		self.rpy = [0,0,0] # CAUTION!!! DONOT CHANGE THIS VALUE IF YOU DONOT KNOW WHAT U ARE DOING
		
		quat = R.from_euler('xyz', [self.rpy[0], self.rpy[0], self.rpy[0]], degrees=True).as_quat()

		self.user_pose_msg.orientation.x = quat[0]
		self.user_pose_msg.orientation.y = quat[1]
		self.user_pose_msg.orientation.z = quat[2]
		self.user_pose_msg.orientation.w = quat[3]
	
	def publish(self):
		quat = R.from_euler('xyz', [self.rpy[0], self.rpy[1], self.rpy[2]], degrees=True).as_quat()

		self.user_pose_msg.orientation.x = quat[0]
		self.user_pose_msg.orientation.y = quat[1]
		self.user_pose_msg.orientation.z = quat[2]
		self.user_pose_msg.orientation.w = quat[3]
		self.user_pose_pub.publish(self.user_pose_msg)

	def help(self):
		print("\n j, l: X +- 0.01m")
		print("\n i, k: Y +- 0.01m")
		print("\n q, w: Z +- 0.01m")
		print("\n u, o: Yaw +- 5 degs")
		print("\n h: Print this")

if __name__ == '__main__':
	rospy.init_node('user_pose_input', anonymous=True)
	user_in = UserPoseInput()

	while not rospy.is_shutdown():
		try:
			ch = getch.getch()

			if ch == 'j':
				user_in.user_pose_msg.position.x += 0.01
			elif ch == 'l':
				user_in.user_pose_msg.position.x -= 0.01
			elif ch == 'i':
				user_in.user_pose_msg.position.y += 0.01
			elif ch == 'k':
				user_in.user_pose_msg.position.y -= 0.01
			elif ch == 'q':
				user_in.user_pose_msg.position.z += 0.01
			elif ch == 'w':
				user_in.user_pose_msg.position.z -= 0.01
			elif ch == 'u':
				user_in.rpy[2] += 5
			elif ch == 'o':
				user_in.rpy[2] -= 5
			elif ch == 'h':
				user_in.help()
				
			user_in.publish()
		except:
			break