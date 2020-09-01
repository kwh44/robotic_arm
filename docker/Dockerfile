FROM lyubomyrd/ros2_no_nvidia

RUN set -x && \
    echo "Upgrading Gazebo 9 to latest release" && \
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get remove -y gazebo* && \
    apt-get install -y libgazebo9 


RUN apt-get update && apt-get install -y \
	ros-$ROS_DISTRO-rttest \
	ros-$ROS_DISTRO-rclcpp-action \
	ros-$ROS_DISTRO-gazebo-dev \
	ros-$ROS_DISTRO-gazebo-msgs \
	ros-$ROS_DISTRO-gazebo-plugins \
	ros-$ROS_DISTRO-gazebo-ros \
	ros-$ROS_DISTRO-gazebo-ros-pkgs

RUN apt-get install -y \
	python3-pip python3-vcstool python3-numpy wget python3-pyqt5 python3-colcon-common-extensions git
RUN apt-get update && apt-get install -y python3-opencv cmake libopenmpi-dev python3-dev zlib1g-dev

RUN mkdir -p ~/ros2_mara_ws/src && cd ~/ros2_mara_ws  && \    
    wget https://raw.githubusercontent.com/acutronicrobotics/MARA/dashing/mara-ros2.repos && \
    vcs import src < mara-ros2.repos && cd ~/ros2_mara_ws/src/HRIM && \
    pip3 install hrim && hrim generate models/actuator/servo/servo.xml && \
    hrim generate models/actuator/gripper/gripper.xml 