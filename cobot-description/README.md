# cobot-description

## About

Contains files and models describing the cobot and the sensors attached to it. Currently, **ur5** cobot has been described in this package.

## Cobot: ur5

It has the following components attached:

1. Gripper (AG95)
2. Camera mount

## Usage
1. To view the URDF model of the cobot
	```bash
	roslaunch cobot-description display.launch
	```

### MoveIt Installation
1. 
	```bash
	rosdep update
	```
2. 
	```bash
	sudo apt-get update
	```
3. 
	```bash
	sudo apt-get dist-upgrade
	```
4. 
	```bash
	sudo apt install ros-noetic-moveit
	```
5. 
	```bash
	sudo apt install ros-noetic-joint-trajectory-controllers
	```
6. 
	```bash
	sudo apt install ros-noetic-moveit-visual-tools
	```

<br/>

[Back to parent navigation](../README.md#navigate)