#!/bin/bash

# locale  # check for UTF-8

# apt update && apt install locales -y
# locale-gen en_US en_US.UTF-8
# update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# export LANG=en_US.UTF-8



# apt install software-properties-common -y
# add-apt-repository universe -y
# apt update &&  apt install curl gnupg2 lsb-release -y
# curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update
apt upgrade -y

apt-get install ros-humble-image-proc ros-humble-camera-info-manager ros-humble-stereo-image-proc ros-humble-tracetools-image-pipeline ros-humble-udp-msgs ros-humble-rviz-common ros-humble-geographic-msgs ros-humble-angles ros-humble-osrf-testing-tools-cpp ros-humble-ament-cmake-google-benchmark ros-humble-random-numbers ros-humble-ros-testing ros-humble-point-cloud-msg-wrapper ros-humble-automotive-platform-msgs ros-humble-apex-test-tools ros-humble-pcl-ros ros-humble-joy-linux ros-humble-sensor-msgs-py ros-humble-gps-msgs ros-humble-nmea-msgs ros-humble-gps-tools ros-humble-can-msgs ros-humble-tf-transformations ros-humble-ament-cmake-nose ros-humble-rmw-cyclonedds-cpp ros-humble-ros2-socketcan ros-humble-rosbag2-storage-mcap ros-humble-robot-upstart -y

apt-get install curl zip unzip tar python3-colcon-common-extensions ros-dev-tools python3-pip libalglib-dev -y && rm -rf /var/lib/apt/lists/*
