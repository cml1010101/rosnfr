ROS_DISTRO=humble
distro=0
export DIR=$(pwd)
set -e
sudo apt update
sudo apt install -y nvidia-jetpack
sudo systemctl restart docker
sudo usermod -aG docker $USER
newgrp docker
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