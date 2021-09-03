# cobot-env/src/ur_collisions

## About

- Gazebo subscriber node that listens to topic `~/physics/contacts` for potential collisions of various environmental objects with UR5.
- Publishes a ROS empty message onto topic `/ur5_collided` indicating that some part of the UR5 has collided with something or is in collision with something.