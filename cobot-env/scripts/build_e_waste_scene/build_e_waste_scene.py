#!/usr/bin/python3

import rospy
import sys
from rospkg import RosPack
sys.path.insert(1, RosPack().get_path('cobot-env') + "/scripts/build_scene/")
from build_scene import MeshSpawner
import math

if __name__ == '__main__':
	rospy.init_node('mesh_spawner_test', anonymous=True)

	spawner = MeshSpawner(meshes_dir='/meshes/e-waste-setup/')
	x_offset = math.sqrt( (1.21/2)**2 + (0.91/2)**2)
	print(spawner.attach_mesh(
			'e_waste_table', 
			'base', 
			[x_offset,0.2,-0.95,90,0,-135],
			'base',
			[],
			'e-waste-setup-rviz.STL'))