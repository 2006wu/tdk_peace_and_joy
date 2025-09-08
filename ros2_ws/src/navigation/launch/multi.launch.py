from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. navigation 的 chassis_controller
        Node(
            package='navigation',
            executable='chassis_controller',
            name='chassis_controller'
        ),

        # 2. realsense2_camera 官方 launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                )
            )
        ),

        # 3. 你的 stm_logger.py (直接 python 執行)
        ExecuteProcess(
            cmd=['python3', os.path.expanduser('~/tdk_peace_and_joy/ros2_ws/src/tool/stm_logger.py')],
            output='screen'
        ),

        # 4. camera 的 launch.xml
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('camera'),
                    'launch',
                    'launch.xml'
                )
            )
        ),
    ])
