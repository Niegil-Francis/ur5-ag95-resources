#!/usr/bin/python3

# ROS imports
import rospy
from rospkg import RosPack

# Message imports
import gazebo_msgs.msg
import geometry_msgs.msg

# misc imports
import pandas as pd

class HumanMotion:
	"""
	A utility class containing methods that aid in moving human in Gazebo.
	"""

	def __init__(self, model_name='human_drone'):
		"""
		Initializes the class

		Parameter
		---------
		human_name: Name of the human model spawned
		"""
		self.human_motion = None

		rp = RosPack()
		file_path = rp.get_path('cobot-env') + '/scripts/human_ctrl/human_motion.txt'

		self.human_motion = pd.read_csv(file_path)
		self.human_motion=self.human_motion[["field.pose.position.x", "field.pose.position.y", "field.pose.position.z",
											"field.pose.orientation.x", "field.pose.orientation.y", "field.pose.orientation.z",
											"field.pose.orientation.w"]]
		
		self.current_row = 0
		self.total_rows = len(self.human_motion)
		self.model_state_pub = rospy.Publisher('/gazebo/set_model_state', gazebo_msgs.msg.ModelState, queue_size=1)

		self.model_state_msg = gazebo_msgs.msg.ModelState()		# model_name,pose,twist,reference_frame
		self.model_state_msg.model_name = model_name
		self.model_state_msg.reference_frame = ''
		self.model_state_msg.twist = geometry_msgs.msg.Twist()

	def set_human_pose(self):
		if self.current_row < self.total_rows:
			self.model_state_msg.pose.position.x = self.human_motion["field.pose.position.x"].iloc[self.current_row]
			self.model_state_msg.pose.position.y = self.human_motion["field.pose.position.y"].iloc[self.current_row]
			self.model_state_msg.pose.position.z = self.human_motion["field.pose.position.z"].iloc[self.current_row]

			self.model_state_msg.pose.orientation.x = self.human_motion["field.pose.orientation.x"].iloc[self.current_row]
			self.model_state_msg.pose.orientation.y = self.human_motion["field.pose.orientation.y"].iloc[self.current_row]
			self.model_state_msg.pose.orientation.z = self.human_motion["field.pose.orientation.z"].iloc[self.current_row]
			self.model_state_msg.pose.orientation.w = self.human_motion["field.pose.orientation.w"].iloc[self.current_row]

			self.model_state_pub.publish(self.model_state_msg)
			self.current_row += 1
		else:
			self.current_row = 0

if __name__ == '__main__':
	rospy.init_node('human_motion', anonymous=True)
	temp = HumanMotion()
	rate = rospy.Rate(50)

	while not rospy.is_shutdown():
		temp.set_human_pose()
		rate.sleep()
	
	temp.motion_data_file.close()