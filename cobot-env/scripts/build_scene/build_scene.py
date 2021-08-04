#!/usr/bin/python3

# ROS imports
import rospy
from rospkg import RosPack

# MoveIt imports
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# misc imports
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

class MeshSpawner:
	"""
	A utility class containing methods that aid in spawning meshes into RViz
	"""

	def __init__(self, ros_pkg_name='cobot-env', meshes_dir='/meshes/'):
		"""
		Initializes the class

		Parameter
		---------
		ros_pkg_name: Name of ROS package containing the directory containing meshes
		meshes_dir: Directory path relative to the package location. Must start and end with '/'
		"""
		group_name = "manipulator"

		moveit_commander.roscpp_initialize(sys.argv)
		self.scene = moveit_commander.PlanningSceneInterface()
		self.move_group = moveit_commander.MoveGroupCommander(group_name)
		rospy.sleep(2)		# sleeping for 2 seconds
		print("moveit_commander initialized")

		# initializing variables
		rp = RosPack()
		self.meshes_dir_path = rp.get_path(ros_pkg_name) + meshes_dir

	def attach_mesh(self, mesh_name, 
						frame_id,
						mesh_pose,
						attach_to,
						touch_links,
						file_name,
						timeout = 4):
		"""
		Attaches a mesh to a link in RViz

		Parameters
		---------
		mesh_name: Name of the object
		frame_id: Frame wrt mesh must be added
		mesh_pose: Mesh pose relative to the frame_id (m, deg) 
				   (6x1 array: x,y,z,roll,pitch,yaw) (sequence: XYZ)
		attach_to: Link name to which mesh must be attached
		touch_links: Array of strings to which attached mesh touches (put [] if none)
		file_name: Name of the mesh file located in the meshes_dir
		timeout: Time (in seconds) for which node waits to check if scene is updated or not

		Returns
		-------
		True if mesh is successfully spawned, else False
		"""
		# remove the object if it exists
		self.scene.remove_world_object(mesh_name)

		self.wait_for_state_update(mesh_name)

		new_pose = geometry_msgs.msg.PoseStamped()
		new_pose.header.frame_id = frame_id
		new_pose.pose.position.x = mesh_pose[0]
		new_pose.pose.position.y = mesh_pose[1]
		new_pose.pose.position.z = mesh_pose[2]
		
		quat = R.from_euler('xyz', [mesh_pose[3], mesh_pose[4], mesh_pose[5]], degrees=True).as_quat()

		new_pose.pose.orientation.x = quat[0]
		new_pose.pose.orientation.y = quat[1]
		new_pose.pose.orientation.z = quat[2]
		new_pose.pose.orientation.w = quat[3]

		self.scene.attach_mesh(attach_to, mesh_name, new_pose, 
								self.meshes_dir_path + file_name, 
								touch_links=touch_links)

		return self.wait_for_state_update(mesh_name, mesh_is_attached=True, timeout=timeout)
	
	def wait_for_state_update(
		self, mesh_name, mesh_is_spawned=False, mesh_is_attached=False, timeout=4
		):
		"""
		Checks the scene update state
		"""
		start = rospy.get_time()
		seconds = rospy.get_time()
		while (seconds - start < timeout) and not rospy.is_shutdown():
			attached_objects = self.scene.get_attached_objects([mesh_name])
			is_attached = len(attached_objects.keys()) > 0

			is_spawned = mesh_name in self.scene.get_known_object_names()

			# Test if we are in the expected state
			if (mesh_is_attached == is_attached) and (mesh_is_spawned == is_spawned):
				return True

			# Sleep so that we give other threads time on the processor
			rospy.sleep(0.1)
			seconds = rospy.get_time()

		return False
	

if __name__ == '__main__':
	rospy.init_node('mesh_spawner_test', anonymous=True)

	spawner = MeshSpawner()
	print(spawner.attach_mesh(
			'table', 
			'base', 
			[0,0,-0.01,0,0,0],
			'base',
			[],
			'table_complete.STL'))