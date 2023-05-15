#!/bin/bash
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
echo "export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/" >> ~/.bashrc
source ~/.bashrc
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline
git clone --recurse-submodules https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox && \
    cd isaac_ros_nvblox && git lfs pull
cd ${ISAAC_ROS_WS}/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference
git clone https://github.com/IntelRealSense/realsense-ros.git -b 4.51.1
cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts && \
    touch .isaac_ros_common-config && \
    echo CONFIG_IMAGE_KEY=ros2_humble.realsense > .isaac_ros_common-config
cd ${ISAAC_ROS_WS}/src
git clone --recursive https://github.com/ThadHouse/CmakeWpilib.git
mkdir -p CmakeWpilib/build && \
    cd CmakeWpilib/build && \
    cmake .. && \
    make && \
    sudo make install
cd ${ISAAC_ROS_WS}/src
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
git clone https://github.com/ros-planning/moveit2.git -b $ROS_DISTRO
# TODO: git clone ROSNTClient
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
    ./scripts/run_dev.sh
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd /workspaces/isaac_ros-dev && \
    colcon build --symlink-install && \
    source install/setup.bash
exit
cd ${HOME}
exit 0