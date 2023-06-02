from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_desciption():
    return LaunchDescription([
        Node(
            package = "usb_cam",
            plugin = "usb_cam::UsbCamNode",
            name = "cam_forward",
            namespace = "cam_forward",
            parameters = [
                {
                    'video_device': '/dev/video0',
                    'framerate': 60.0,
                    'io_method': 'mmap',
                    'image_width': 1280.0,
                    'image_height': 720.0,
                    'camera_info_url': 'package://nfr-ros/config/forward_camera_info.yaml'
                }
            ]
        ),
        Node(
            package = "isaac_ros_apriltag",
            plugin = "nvidia::isaac_ros::apriltag::AprilTagNode",
            name = "apriltag_forward",
            namespace = "apriltag_forward"
            parameters = [{
                'size': 0.16,
                'max_tags': 64,
                'tag_family': 'tag16h5',
                'camera_frame': 'cam_forward'
            }]
        ),
        Node(
            package = 'nfr-ros',
            plugin = 'NTClientNode',
            parameters = [{
                'apriltag_nodes': ['apriltag_forward']
            }]
        )
    ])
