# Code to plot the waypoints
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import time

data = pd.read_csv("Graph_data.csv")
names_axis=['upper_arm_link_x','upper_arm_link_y','upper_arm_link_z','shoulder_link_x','shoulder_link_y','shoulder_link_z',
'forearm_link_x','forearm_link_y','forearm_link_z','wrist_1_link_x','wrist_1_link_y','wrist_1_link_z','wrist_2_link_x','wrist_2_link_y','wrist_2_link_z'
,'wrist_3_link_x','wrist_3_link_y','wrist_3_link_z','distance']
names=['upper_arm_link','shoulder_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link']

font = {'size': 28,
        }
font_title = {'size': 33,
}


counter=0
def animate(i):
	global counter
	# plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
	fig = plt.figure(figsize=(15,10))
	ax = plt.axes(projection='3d')
	for count,j in enumerate(names):
		x = data[names_axis[count*3]].values[counter:i]
		y = data[names_axis[count*3+1]].values[counter:i]
		z = data[names_axis[count*3+2]].values[counter:i]
		scatter=ax.scatter(x, y, z, c = data['distance'].values[counter:i], s=2)
		ax.plot(x,y,z,label=j,linewidth=0.9)
	legend1 = ax.legend(*scatter.legend_elements(num=5),
	loc="upper right", title="Euclidean Angular Distance",fontsize=18)
	ax.legend(loc='upper left',fontsize=22)
	ax.set_xlabel("x(m)",fontsize=22)
	ax.set_ylabel('y(m)',fontsize=22)
	ax.set_zlabel('z(m)',fontsize=22)
	ax.add_artist(legend1)
	plt.savefig("data/images/fig_"+str(i)+'.png')
	if(i>=50):
		counter+=1
	time.sleep(0.5)

for i in range(1,len(data)):
	animate(i)


