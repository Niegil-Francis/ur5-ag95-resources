#!/usr/bin/python3

# ROS imports
import rospy

# Message imports
import gazebo_msgs.msg
import geometry_msgs.msg

class HumanPoseFinder:
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
		self.model_state_pub = rospy.Publisher('/gazebo/set_model_state', gazebo_msgs.msg.ModelState, queue_size=1)
		self.user_pose_sub = rospy.Subscriber('/user_pose', geometry_msgs.msg.Pose, self.user_pose_callback)

		self.model_state_msg = gazebo_msgs.msg.ModelState()		# model_name,pose,twist,reference_frame
		self.model_state_msg.model_name = model_name
		self.model_state_msg.reference_frame = ''
		self.model_state_msg.twist = geometry_msgs.msg.Twist()

	def user_pose_callback(self, data):
		self.set_human_pose(data)

	def set_human_pose(self, pose):
		self.model_state_msg.pose = pose
		self.model_state_pub.publish(self.model_state_msg)
		rospy.loginfo('Position:\n' + str(pose.position) + '\nOrientation:\n' + str(pose.orientation))

if __name__ == '__main__':
	rospy.init_node('human_pose_finder', anonymous=True)
	temp = HumanPoseFinder()

	while not rospy.is_shutdown():
		pass
