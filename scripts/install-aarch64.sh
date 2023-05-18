ROS_DISTRO=humble
distro=0
export DIR=$(pwd)
set -e
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
scripts/install.sh