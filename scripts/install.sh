#!/bin/bash
ROS_DISTRO=humble
if [$(uname -p) == x86_64]; then
    echo "Downloading ISAAC ROS for an x86-64 system"
    echo "Assuming nvidia is present"
    distro=0
elif [$($(uname -p) == aarch64) || $(uname -p == arm64)]; then
    echo "Downloading ISAAC ROS for an arm64 system"
    echo "Assuming nvidia is present"
    distro=1
else
    echo "Unidentified system. Halt."
    exit 1
fi
set -e
cd ${HOME}
if [distro == 0]; then
    sudo apt update
    sudo apt install -y nvidia-jetpack
    sudo systemctl restart docker
    sudo usermod -aG docker $USER
    newgrp docker
    sudo apt install -y curl
    curl https://raw.githubusercontent.com/cml1010101/rosnfr/main/resources/arm64-daemon.json > arm64-daemon.json
    sudo rm -rf /etc/docker/daemon.json
    sudo mv arm64-daemon.json /etc/docker/daemon.json
    sudo systemctl daemon-reload && sudo systemctl restart docker
elif [distro == 1]: then
    sudo apt update
    sudo apt install -y curl
    curl -s -L https://nvidia.github.io/nvidia-container-runtime/gpgkey | \
        sudo apt-key add - distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    curl -s -L https://nvidia.github.io/nvidia-container-runtime/$distribution/nvidia-container-runtime.list | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-runtime.list
    sudo apt update
    sudo apt install -y nvidia-container-runtime
    sudo apt install -y nvidia-docker2
    curl https://raw.githubusercontent.com/cml1010101/rosnfr/main/resources/x86_64-daemon.json > x86_64-daemon.json
    sudo rm -rf /etc/docker/daemon.json
    sudo mv x86_64-daemon.json /etc/docker/daemon.json
    sudo pkill -SIGHUP dockerd
    sudo systemctl daemon-reload && sudo systemctl restart docker
fi
sudo apt install -y git-lfs
git lfs install --skip-repo
mkdir -p ~/workspaces/isaac_ros-dev/src
echo "export ISAAC_ROS_WS=$(pwd)" >> ~/.bashrc
source ~/.bashrc
# TODO: git clone ROSNTClient
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
    ./scripts/run_dev.sh
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
cd ${HOME}
exit 0