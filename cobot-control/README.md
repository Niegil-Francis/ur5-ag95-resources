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

### Basic picking and placing
> This ROS node is only capable of identifying RED or GREEN boxes in the field of view of the camera, picks it and places at the yellow corner of the table.
1. Make sure the drivers are initialized, `ros_interface.urp` running and the `calibrate_camera.launch` file running. Also, the `launch_camera_topics` parameter is the `spawn_controllers.launch` file must be set to `false` or else the node cannot access the camera data.
2. 
    ```bash
    rosrun cobot-control basic_pick_place.py
    ```

### Adv picking and placing
> This ROS node is only capable of identifying RED or GREEN boxes in the field of view of the camera, picks it and places at the yellow corner of the table. This node subscribes to the camera topics while the previous node access the camera data directly.
1. Make sure the drivers are initialized, `ros_interface.urp` running and the `calibrate_camera.launch` file running. Also, the `launch_camera_topics` parameter is the `spawn_controllers.launch` file must be set to `true` or else the node cannot access the camera data.
2. 
    ```bash
    rosrun cobot-control adv_pick_place.py
    ```

<br/>

[Back to parent navigation](../README.md#navigate)