# traj_replace

## About
- Replaces a defined trajectory with another trajectory (TODO)
- Scales the velocity of a pre-defined trajectory by a factor between 0 to 1 both statically and dynamically (TODO)
- Execute each line in separate terminals
	```bash
	roslaunch cobot-control ur5_gazebo.launch
	rosrun cobot-control traj_replace_gui
	rosrun cobot-control traj_replace.py
	```
- `traj_replace_gui` publishes:
	| Topic name | Message type | About |
	| --- | --- | --- |
	| traj_type | `std_msgs/Int8` | User's command to execute or stop a trajectory type. Does not work with *Home* and *Start pose* trajectory types. **Trajectory types:** *STOP* (0), *Home* (1), *Start pose* (2), *Replace trajectory* (3), *Scale velocity* (4) |

- `traj_replace.py` subscribes to all the topics published by `traj_replace_gui`