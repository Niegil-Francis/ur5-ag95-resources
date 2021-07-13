# ur5-ag95-resources

## Repository Overview
The **ur5-ag95-resources** is a compilation of ROS packages that aid in the simulation and control of UR5.

### Key features
- Tested on ROS-Noetic + Gazebo-11
- Only the cobot **UR5** has been implemented yet

### Packages
1. [cobot-control] : Contains files related to controlling the cobot. It is the main ROS package from which launch files realted to testing and final implementation are executed.
2. [cobot-description] : Contains files and models describing the cobot and the sensors attached to it.
3. [cobot-env] : Contains files and models describing the environment.
4. [ur5_ag95_moveit_config] : Contains files for controlling the **UR5** cobot using *MoveIt*.


### Navigate
1. [cobot-control](./cobot-control)
2. [cobot-description](./cobot-description)
3. [cobot-env](./cobot-env)
4. [ur5_ag95_moveit_config](./ur5_ag95_moveit_config)

[cobot-control]: ./cobot-control/README.md
[cobot-description]: ./cobot-description/README.md
[cobot-env]: ./cobot-env/README.md
[ur5_ag95_moveit_config]: ./ur5_ag95_moveit_config/README.md