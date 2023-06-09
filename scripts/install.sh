#!/bin/bash
sudo apt install -y git-lfs
git lfs install --skip-repo
echo "export ISAAC_ROS_WS=$(pwd)" >> ~/.bashrc
source ~/.bashrc
git submodule update --init --recursive
docker login nvcr.io $oauthtoken
echo $ISAAC_ROS_WS
docker pull cml1010101/rosnfr:v8
sudo cp services/rosnfr.service /etc/systemd/system/
sudo mkdir -p /opt/rosnfr
sudo cp scripts/launch.sh /opt/rosnfr/
sudo systemctl enable rosnfr.service
exit 0
