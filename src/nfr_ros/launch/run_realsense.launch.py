from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
def generate_launch_description():
    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify',
        namespace='',
        parameters=[{
            'output_width': 1920,
            'output_height': 1080,
        }]
    )
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='',
        parameters=[{
            'tag_family': 'tag16h5'
        }]
    )
    realsense_camera_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='realsense2_camera',
        namespace='',
        parameters=[{
            'color_height': 1080,
            'color_width': 1920,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_depth': False,
        }],
        remappings=[('color/image_raw', 'image'),
                    ('color/camera_info', 'camera_info')]
    )
    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            rectify_node,
            apriltag_node,
            realsense_camera_node
        ],
        output='screen'
    )
    return LaunchDescription([
        apriltag_container
    ])