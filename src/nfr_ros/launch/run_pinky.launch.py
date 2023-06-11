from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            actions = [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('nfr_ros'), 'launch'),
                        '/run_realsense.launch.py'
                    ])
                )
            ]
        ),
        Node(
            package='nfr_ros',
            name='ntclient',
            executable='ntclient',
            parameters=[{
                'apriltag_interface_namespaces': [
                    ''
                ]
            }]
        )
    ])