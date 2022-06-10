import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('cslam'),
        'config',
        'cslam/'
        )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/r0',
                              description=''),
        DeclareLaunchArgument('config_file', default_value='default.yaml',
                              description=''),
        DeclareLaunchArgument('config',
                              default_value=[
                                    config_path,
                                    LaunchConfiguration('config_file')
                              ],
                              description=''),
        DeclareLaunchArgument(
            'launch_prefix',
            default_value='',
            description=
            'For debugging purpose, it fills prefix tag of the nodes, e.g., "xterm -e gdb -ex run --args"'
        ),

        Node(package='cslam',
             executable='loop_closure_detection.py',
             name='cslam_loop_closure_detection',
             parameters=[LaunchConfiguration('config')],
             namespace=LaunchConfiguration('namespace')),

        Node(package='cslam',
             executable='map_data_handler',
             name='cslam_map_data_handler',
             parameters=[LaunchConfiguration('config')],
             prefix=LaunchConfiguration('launch_prefix'),
             namespace=LaunchConfiguration('namespace')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('cslam'),
                'launch',
                'third_party',
                'rtabmap_mapping.launch.py'
                )),
            launch_arguments={}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('cslam'),
                'launch',
                'third_party',
                'rtabmap_odometry.launch.py'
                )),
            launch_arguments={}.items(),
         ),

    ])