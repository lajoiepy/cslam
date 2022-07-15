import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    loop_detection_node = Node(package='cslam',
                               executable='loop_closure_detection_node.py',
                               name='cslam_loop_closure_detection',
                               parameters=[
                                   LaunchConfiguration('config'), {
                                       "robot_id":
                                       LaunchConfiguration('robot_id'),
                                       "nb_robots":
                                       LaunchConfiguration('nb_robots'),
                                   }
                               ],
                               namespace=LaunchConfiguration('namespace'))

    map_manager_node = Node(package='cslam',
                            executable='map_manager',
                            name='cslam_map_manager',
                            parameters=[
                                LaunchConfiguration('config'), {
                                    "robot_id":
                                    LaunchConfiguration('robot_id'),
                                    "nb_robots":
                                    LaunchConfiguration('nb_robots'),
                                }
                            ],
                            prefix=LaunchConfiguration('launch_prefix'),
                            namespace=LaunchConfiguration('namespace'))

    pose_graph_manager_node = Node(package='cslam',
                                   executable='pose_graph_manager',
                                   name='cslam_pose_graph_manager',
                                   parameters=[
                                       LaunchConfiguration('config'), {
                                           "robot_id":
                                           LaunchConfiguration('robot_id'),
                                           "nb_robots":
                                           LaunchConfiguration('nb_robots'),
                                       }
                                   ],
                                   prefix=LaunchConfiguration('launch_prefix'),
                                   namespace=LaunchConfiguration('namespace'))

    return [
        loop_detection_node,
        map_manager_node,
        pose_graph_manager_node,
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/r0',
                              description=''),
        DeclareLaunchArgument('robot_id', default_value='0', description=''),
        DeclareLaunchArgument('nb_robots', default_value='1', description=''),
        DeclareLaunchArgument('config_path',
                              default_value=os.path.join(
                                  get_package_share_directory('cslam'),
                                  'config', 'cslam/'),
                              description=''),
        DeclareLaunchArgument('config_file',
                              default_value='default_kitti.yaml',
                              description=''),
        DeclareLaunchArgument('config',
                              default_value=[
                                  LaunchConfiguration('config_path'),
                                  LaunchConfiguration('config_file')
                              ],
                              description=''),
        DeclareLaunchArgument(
            'launch_prefix',
            default_value='',
            description=
            'For debugging purpose, it fills prefix tag of the nodes, e.g., "xterm -e gdb -ex run --args"'
        ),
        DeclareLaunchArgument('log_level',
                              default_value='error',
                              description=''),
        OpaqueFunction(function=launch_setup),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('cslam'), 'launch',
                             'slam', 'rtabmap_odometry.launch.py')),
            launch_arguments={
                'log_level': LaunchConfiguration('log_level'),
                "robot_id": LaunchConfiguration('robot_id'),
                "nb_robots": LaunchConfiguration('nb_robots'),
            }.items(),
        )
    ])
