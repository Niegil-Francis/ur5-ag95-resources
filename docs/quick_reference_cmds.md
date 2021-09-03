# Quick Reference Commands

### Real-world control with MoveIt

```bash
# Terminal 1:
roslaunch ur-ros-driver-config spawn_controllers.launch launch_gripper_driver:=true 

# Terminal 2:
roslaunch cobot-control ur5_moveit_control.launch only_sim:=false
```

### Gazebo control without MoveIt planning

#### Launching simulation environment

```bash
roslaunch cobot-control ur5_gazebo.launch scene:=e_waste
```

> `scene` can take either `empty_table` or `e_waste`.

#### Moving human model in predefined path on loop

```bash
rosrun cobot-env human_motion.py
```

#### Controlling human motion via keyboard

```bash
# Terminal 1:
rosrun cobot-env pose_publisher.py

# Terminal 2:
rosrun cobot-env user_pose_input.py
```

#### Learning whether UR5 collided with an object in the environment

```bash
rosrun cobot-env ur_collisions
```

- Publishes an empty message onto topic `/ur5_collided` whenever there is collision with UR5

<br/>

[Back to parent navigation](../README.md#navigate)