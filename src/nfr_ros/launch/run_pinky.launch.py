from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            actions = [
                PushRosNamespace('/realsense_cam'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(get_package_share_directory('nfr_ros'), 'launch'),
                        '/run_realsense.launch.py'
                    ])
                )
            ]
        ),
        # GroupAction(
        #     actions = [
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource([
        #                 os.path.join(get_package_share_directory('nav2_bringup'), 'launch'),
        #                 '/navigation_launch.py'
        #             ]),
        #             launch_arguments=[
        #                 ('params_file', os.path.join(get_package_share_directory('nfr_ros'), 'launch', 'nav2_params.yaml'))
        #             ]
        #         )
        #     ]
        # ),
        Node(
            package='nfr_ros',
            name='ntclient',
            executable='ntclient',
            parameters=[{
                'apriltag_interface_namespaces': [
                    'realsense_cam'
                ]
            }]
        )
    ])