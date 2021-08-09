# stop_resume_exe

## About

Two separate python scripts implement this control functionality:
1. `stop_resume_gui.py`: Provides a user interface to control the stop/resume operations of the cobot. The `-s` option is specified when executing this script in **only simulation** mode (i.e., using only RViz). Publishes messages to the topic: `/is_ur5_resumed`.
2. `stop_resume_traj.py`: Used to create custom trajectory which is then executed as an *interruptable trajectory*. Subscribes to the topic: `/is_ur5_resumed`.

The `topics.yaml` file contains the topics that are published/subscribed by the scripts.

## Usage
Both the scripts can be executed independently on separate terminals as:
```bash
# for only RViz simulation
rosrun cobot-control stop_resume_gui.py -s 
# or for hardware
rosrun cobot-control stop_resume_gui.py

# for both RViz simulation and hardware
rosrun cobot-control stop_resume_traj.py
```

### References
- [Understanding Trajectory Replacement](http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement)
