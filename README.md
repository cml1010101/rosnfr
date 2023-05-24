# rosnfr

This is a repository that provides utility scripts for installing ISAAC ROS and other relavant packages on a Nvidia Xavier NX.

## Installation Instructions

After cloning the repo, run the archetecture specific install script.

### On x86_64

`chmod +x scripts/install-x86_64.sh && scripts/install-x86_64.sh`

### On Aarch64

`chmod +x scripts/install-aarch64.sh && scripts/install-aarch64.sh`

Afterwards, run the common installation script.

`chmod +x scripts/install.sh && scripts/install.sh`
