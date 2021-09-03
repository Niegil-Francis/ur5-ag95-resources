# cobot-env/scripts/human_ctrl

## About

Contains scripts that aid in controlling the human motion.

| File Name | About |
| --- | --- |
| `human_motion.py` | Used to play the recorded positions from `human_motion.txt` |
| `human_motion.txt` | CSV file generated from `rosbag` | 
| `pose_publisher.py` | Publishes to topic `/gazebo/set_model_state` and subscribes to `/user_pose` |
| `user_pose_input.py` | Publishes to topic `/user_pose` by taking keyboard control |