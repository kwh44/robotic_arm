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
- cd /root; git clone https://github.com/kwh44/robotic_arm
- cd ~/robotic_arm/src
- python3 arm_control.py   # controlling joints and gripper
- python3 spawn_object.py  # add coke can and beer bottle to Gazebo arm simulation
- python3 read_cam_frame.py # read video feed from camera on the arm