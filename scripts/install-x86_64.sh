ROS_DISTRO=humble
distro=0
export DIR=$(pwd)
set -e
sudo apt update
sudo apt install -y curl
curl -s -L https://nvidia.github.io/nvidia-container-runtime/gpgkey | \
    sudo apt-key add -
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
scripts/install.sh