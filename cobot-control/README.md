# cobot-control

## About
Contains files related to controlling the cobot. It is the main ROS package from which launch files related to testing and final implementation are executed.

## Usage

### Controlling real-world UR5 via RViz
1. Make sure the drivers are initialized and `ros_interface.urp` is running on the UR5 polyscope.

2. 
    ```bash
    roslaunch cobot-control ur5_moveit_control.launch
    ```

### Moving gripper onto the object to be picked
1. Make sure the drivers are initialized, `ros_interface.urp` running and the `calibrate_camera.launch` file running.
2. 
    ```bash
    rosrun cobot-control moveToObj.py
    ```


<br/>

[Back to parent navigation](../README.md#navigate)