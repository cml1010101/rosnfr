cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
    ./scripts/run_dev.sh
# Uncomment the camera you are using, either a USB Camera or the Intel Realsense
ros2 launch isaac_ros_apriltag isaac_ros_apriltag_usb_cam.launch.py
# ros2 launch isaac_ros_apriltag isaac_ros_apriltag_realsense.launch.py