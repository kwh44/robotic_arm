# Robotic Arm

### Short video with RL agent training
[Youtube Video Link](https://youtu.be/p1741WDXQwM)
### Install and run (Linux Guide)
##### Open terminal session
###### Start docker running
```shell script
docker pull kwh44/mara_robotic_arm
xhost +local:root
docker run -it \
    --privileged \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    --device=/dev/dri:/dev/dri \
    --env="DISPLAY=$DISPLAY" \
    --env QT_X11_NO_MITSHM=1 \
    kwh44/mara_robotic_arm:latest
```
##### Inside docker container
###### Start Gazebo simulator
```shell script
ros2 launch mara_gazebo mara.launch.py --urdf mara_robot_gripper_140_camera_train > /dev/null &
```
###### Train RL agent
```shell script
python3 robotic_arm/src/gym-robotic-arm.py 
```