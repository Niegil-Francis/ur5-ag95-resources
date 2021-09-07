# cobot-env/src/ur_collisions

## About

- Gazebo subscriber node that listens to topic `~/physics/contacts` for potential collisions of various environmental objects with UR5.
- Publishes a ROS Int32 message onto topic `/ur5_collided` indicating the number of collisions with UR5.