#!/bin/bash
sudo apt install -y git-lfs
git lfs install --skip-repo
echo "export ISAAC_ROS_WS=$(pwd)" >> ~/.bashrc
source ~/.bashrc
git submodule update --init --recursive
docker login nvcr.io $oauthtoken
echo $ISAAC_ROS_WS
#cd src/isaac_ros_common && \
#    ./scripts/run_dev.sh $ISAAC_ROS_WS
docker pull cml1010101/rosnfr:v8
exit 0
