# ur5-ag95-resources

## Repository Overview
The **ur5-ag95-resources** is a compilation of ROS packages that aid in the simulation and control of UR5. 


### Key features
- Tested on ROS-Melodic + Gazebo-9 and ROS-Noetic + Gazebo-11

### Packages
1. [cobot-control] : Contains files related to controlling the cobot. It is the main ROS package from which launch files realted to testing and final implementation are executed.
2. [cobot-description] : Contains files and models describing the cobot and the sensors attached to it.
3. [cobot-env] : Contains files and models describing the environment.
4. [ur5_ag95_moveit_config] : Contains files for controlling the **UR5** cobot using *MoveIt*.

### Dependencies
Nodes and launch files of this package require packages of the _ROS-Drivers_ repository:
```bash
git clone --recursive https://github.com/Robotics-Innovations-Lab/ROS-Drivers.git
```
Install the python dependencies by executing the following command in the root directory of this repository
```bash
pip install -r requirements.txt
```

### Navigate
1. [cobot-control](./cobot-control)
2. [cobot-description](./cobot-description)
3. [cobot-env](./cobot-env)
4. [ur5_ag95_moveit_config](./ur5_ag95_moveit_config)
5. [MoveIt installation instructions](./cobot-description/README.md#moveit-installation)
6. [Quick Reference Commands](./docs/quick_reference_cmds.md)

[cobot-control]: ./cobot-control/README.md
[cobot-description]: ./cobot-description/README.md
[cobot-env]: ./cobot-env/README.md
[ur5_ag95_moveit_config]: ./ur5_ag95_moveit_config/README.md