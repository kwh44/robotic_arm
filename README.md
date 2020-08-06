# Robotic Arm
### Install and run 

##### Open new terminal session (MARA simulation)

- git clone https://github.com/kwh44/robotic_arm
- cd robotic_arm/docker
- chmod +x build.bash run.bash
- ./build.bash
- ./run.bash
- exit # close xterm ;=)

###### Inside docker container

- cd ~/ros2_mara_ws
- source /opt/ros/dashing/setup.bash
- cd ~/ros2_mara_ws
- colcon build --merge-install --packages-skip individual_trajectories_bridge
- source ~/ros2_mara_ws/install/setup.bash
- ros2 launch mara_gazebo mara.launch.py --urdf mara_robot_gripper_140


##### Open new terminal session (Arm control & Object spawn)
- ./run.bash

###### Inside docker container
- cd ~/ros2_mara_ws
- source /opt/ros/dashing/setup.bash
- cd ~/ros2_mara_ws
- colcon build --merge-install --packages-skip individual_trajectories_bridge
- source ~/ros2_mara_ws/install/setup.bash
- cd /root
- git clone https://github.com/kwh44/robotic_arm
- cd ~/robotic_arm/src
- python3 arm_control.py   # controlling joints and gripper
- python3 spawn_object.py  # add coke can and beer bottle to Gazebo arm simulation