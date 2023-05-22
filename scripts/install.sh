#!/bin/bash
sudo apt install -y git-lfs
git lfs install --skip-repo
echo "export ISAAC_ROS_WS=$(pwd)" >> ~/.bashrc
source ~/.bashrc
docker login nvcr.io $oauthtoken
touch src/isaac_ros_common/scripts/.isaac_ros_common-config
echo "CONFIG_IMAGE_KEY=ros2_humble.realsense" >> src/isaac_ros_common/scripts.isaac_ros_common-config
cd src/isaac_ros_common && \
    ./scripts/run_dev.sh $ISAAC_ROS_WS
exit 0
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libbullet-dev \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget
python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest
sudo apt install -y ros-humble-navigation2
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
rosdep install --from-paths src --ignore-src --rosdistro galactic -y --skip-keys="console_bridge fastcdr fastrtps libopensplice67 libopensplice69 libopensplice69-dev libopensplice-cpp-dev python3-lark-parser python3-lark python3-babeltrace python3-psutil python3-lark python3-babeltrace python3-psutil python3-netifaces python3-rclpy python3-rosbag2 python3-rosbag2-storage python3-rosbag2-storage-default-plugins python3-ament-pylint"
mkdir -p CmakeWpilib/build && \
    cd CmakeWpilib/build && \
    cmake .. && \
    make && \
    sudo make install
cd /workspaces/isaac_ros-dev && \
    colcon build --symlink-install && \
    source install/setup.bash
source /workspaces/install/setup.bash
exit
# cd ${HOME}
exit 0
