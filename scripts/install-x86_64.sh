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
sudo tee /etc/docker/daemon.json <<EOF
{
    "runtimes": {
        "nvidia": {
            "path": "/usr/bin/nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
}
EOF
sudo pkill -SIGHUP dockerd
sudo systemctl daemon-reload && sudo systemctl restart docker
scripts/install.sh