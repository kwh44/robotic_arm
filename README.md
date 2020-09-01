# Robotic Arm
### Install and run 

##### Open new terminal session

- git clone https://github.com/kwh44/robotic_arm
- cd robotic_arm/docker
- ./build.bash
- ./run.bash
- exit # close xterm ;=)

###### Inside docker container

- cd ~/ros2_mara_ws
- source /opt/ros/dashing/setup.bash
- colcon build --merge-install --packages-skip individual_trajectories_bridge
- source ~/ros2_mara_ws/install/setup.bash
- ros2 launch mara_gazebo mara.launch.py --urdf mara_robot_gripper_140_camera_train > /dev/null &
- cd /root && git clone https://github.com/kwh44/robotic_arm
- cd ~/robotic_arm/src
- cd model_pose_publisher && colcon build --symlink-install # build gazebo&ros2 entity_state publisher
- python3 gym-robotic-arm.py # start agent learning