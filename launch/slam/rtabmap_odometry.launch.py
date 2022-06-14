import os

from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from typing import Text
from ament_index_python.packages import get_package_share_directory


#Based on https://answers.ros.org/question/363763/ros2-how-best-to-conditionally-include-a-prefix-in-a-launchpy-file/
class ConditionalText(Substitution):

    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: 'LaunchContext') -> Text:
        if self.condition == True or self.condition == 'true' or self.condition == 'True':
            return self.text_if
        else:
            return self.text_else


class ConditionalBool(Substitution):

    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: 'LaunchContext') -> bool:
        if self.condition:
            return self.text_if
        else:
            return self.text_else


def launch_setup(context, *args, **kwargs):

    return [
        DeclareLaunchArgument('depth',
                              default_value=ConditionalText(
                                  'false', 'true',
                                  IfCondition(
                                      PythonExpression([
                                          "'",
                                          LaunchConfiguration('stereo'),
                                          "' == 'true'"
                                      ]))._predicate_func(context)),
                              description=''),
        DeclareLaunchArgument('subscribe_rgb',
                              default_value=LaunchConfiguration('depth'),
                              description=''),
        DeclareLaunchArgument(
            'args',
            default_value=LaunchConfiguration('rtabmap_args'),
            description=
            'Can be used to pass RTAB-Map\'s parameters or other flags like --udebug and --delete_db_on_start/-d'
        ),
        DeclareLaunchArgument(
            'qos_image',
            default_value=LaunchConfiguration('qos'),
            description=
            'Specific QoS used for image input data: 0=system default, 1=Reliable, 2=Best Effort.'
        ),
        DeclareLaunchArgument(
            'qos_camera_info',
            default_value=LaunchConfiguration('qos'),
            description=
            'Specific QoS used for camera info input data: 0=system default, 1=Reliable, 2=Best Effort.'
        ),
        DeclareLaunchArgument(
            'qos_scan',
            default_value=LaunchConfiguration('qos'),
            description=
            'Specific QoS used for scan input data: 0=system default, 1=Reliable, 2=Best Effort.'
        ),
        DeclareLaunchArgument(
            'qos_odom',
            default_value=LaunchConfiguration('qos'),
            description=
            'Specific QoS used for odometry input data: 0=system default, 1=Reliable, 2=Best Effort.'
        ),
        DeclareLaunchArgument(
            'qos_user_data',
            default_value=LaunchConfiguration('qos'),
            description=
            'Specific QoS used for user input data: 0=system default, 1=Reliable, 2=Best Effort.'
        ),
        DeclareLaunchArgument(
            'qos_imu',
            default_value=LaunchConfiguration('qos'),
            description=
            'Specific QoS used for imu input data: 0=system default, 1=Reliable, 2=Best Effort.'
        ),
        DeclareLaunchArgument(
            'qos_gps',
            default_value=LaunchConfiguration('qos'),
            description=
            'Specific QoS used for gps input data: 0=system default, 1=Reliable, 2=Best Effort.'
        ),
        SetParameter(name='use_sim_time',
                     value=LaunchConfiguration('use_sim_time')),
        # 'use_sim_time' will be set on all nodes following the line above

        
        # RGB-D odometry
        Node(package='rtabmap_ros',
             executable='rgbd_odometry',
             output="screen",
             condition=IfCondition(
                 PythonExpression([
                     "'",
                     LaunchConfiguration('icp_odometry'), "' != 'true' and '",
                     LaunchConfiguration('visual_odometry'),
                     "' == 'true' and '",
                     LaunchConfiguration('stereo'), "' != 'true'"
                 ])),
             parameters=[{
                 "frame_id":
                 LaunchConfiguration('frame_id'),
                 "odom_frame_id":
                 LaunchConfiguration('vo_frame_id'),
                 "publish_tf":
                 LaunchConfiguration('publish_tf_odom'),
                 "ground_truth_frame_id":
                 LaunchConfiguration('ground_truth_frame_id').perform(context),
                 "ground_truth_base_frame_id":
                 LaunchConfiguration('ground_truth_base_frame_id').perform(
                     context),
                 "wait_for_transform":
                 LaunchConfiguration('wait_for_transform'),
                 "wait_imu_to_init":
                 LaunchConfiguration('wait_imu_to_init'),
                 "approx_sync":
                 LaunchConfiguration('approx_sync'),
                 "config_path":
                 LaunchConfiguration('cfg').perform(context),
                 "queue_size":
                 LaunchConfiguration('queue_size'),
                 "qos":
                 LaunchConfiguration('qos_image'),
                 "qos_camera_info":
                 LaunchConfiguration('qos_camera_info'),
                 "qos_imu":
                 LaunchConfiguration('qos_imu'),
                 "subscribe_rgbd":
                 LaunchConfiguration('subscribe_rgbd'),
                 "guess_frame_id":
                 LaunchConfiguration('odom_guess_frame_id').perform(context),
                 "guess_min_translation":
                 LaunchConfiguration('odom_guess_min_translation'),
                 "guess_min_rotation":
                 LaunchConfiguration('odom_guess_min_rotation')
             }],
             remappings=[
                 ("rgb/image",
                  ConditionalText(
                      ''.join([
                          LaunchConfiguration('namespace').perform(context),
                          LaunchConfiguration('stereo_namespace').perform(
                              context),
                          LaunchConfiguration('rgb_topic').perform(context),
                          "_relay"
                      ]), ''.join([
                          LaunchConfiguration('namespace').perform(context),
                          LaunchConfiguration('stereo_namespace').perform(
                              context),
                          LaunchConfiguration('rgb_topic').perform(context)
                      ]),
                      LaunchConfiguration('compressed').perform(context))),
                 ("depth/image",
                  ConditionalText(
                      ''.join([
                          LaunchConfiguration('namespace').perform(context),
                          LaunchConfiguration('stereo_namespace').perform(
                              context),
                          LaunchConfiguration('depth_topic').perform(context),
                          "_relay"
                      ]), ''.join([
                          LaunchConfiguration('namespace').perform(context),
                          LaunchConfiguration('stereo_namespace').perform(
                              context),
                          LaunchConfiguration('depth_topic').perform(context)
                      ]),
                      LaunchConfiguration('compressed').perform(context))),
                 ("rgb/camera_info", LaunchConfiguration('camera_info_topic')),
                 ("rgbd_image",
                  ConditionalText(
                      ''.join([
                          LaunchConfiguration('namespace').perform(context),
                          LaunchConfiguration('stereo_namespace').perform(
                              context),
                          LaunchConfiguration('rgbd_topic').perform(context)
                      ]), ''.join([
                          LaunchConfiguration('namespace').perform(context),
                          LaunchConfiguration('stereo_namespace').perform(
                              context),
                          LaunchConfiguration('rgbd_topic').perform(context),
                          "_relay"
                      ]),
                      LaunchConfiguration('rgbd_sync').perform(context))),
                 ("odom", LaunchConfiguration('odom_topic')),
                 ("imu", LaunchConfiguration('imu_topic'))
             ],
             arguments=[
                 LaunchConfiguration("args"),
                 LaunchConfiguration("odom_args"),
                 '--ros-args', '--log-level', 'error'
             ],
             prefix=LaunchConfiguration('launch_prefix'),
             namespace=LaunchConfiguration('namespace')),

        # Stereo odometry
        Node(
            package='rtabmap_ros',
            executable='stereo_odometry',
            output="screen",
            condition=IfCondition(
                PythonExpression([
                    "'",
                    LaunchConfiguration('icp_odometry'), "' != 'true' and '",
                    LaunchConfiguration('visual_odometry'),
                    "' == 'true' and '",
                    LaunchConfiguration('stereo'), "' == 'true'"
                ])),
            parameters=[{
                "frame_id":
                LaunchConfiguration('frame_id'),
                "odom_frame_id":
                LaunchConfiguration('vo_frame_id'),
                "publish_tf":
                LaunchConfiguration('publish_tf_odom'),
                "ground_truth_frame_id":
                LaunchConfiguration('ground_truth_frame_id').perform(context),
                "ground_truth_base_frame_id":
                LaunchConfiguration('ground_truth_base_frame_id').perform(
                    context),
                "wait_for_transform":
                LaunchConfiguration('wait_for_transform'),
                "wait_imu_to_init":
                LaunchConfiguration('wait_imu_to_init'),
                "approx_sync":
                LaunchConfiguration('approx_sync'),
                "config_path":
                LaunchConfiguration('cfg').perform(context),
                "queue_size":
                LaunchConfiguration('queue_size'),
                "qos":
                LaunchConfiguration('qos_image'),
                "qos_camera_info":
                LaunchConfiguration('qos_camera_info'),
                "qos_imu":
                LaunchConfiguration('qos_imu'),
                "subscribe_rgbd":
                LaunchConfiguration('subscribe_rgbd'),
                "guess_frame_id":
                LaunchConfiguration('odom_guess_frame_id').perform(context),
                "guess_min_translation":
                LaunchConfiguration('odom_guess_min_translation'),
                "guess_min_rotation":
                LaunchConfiguration('odom_guess_min_rotation'),
                # Ensure that we don't lose tracking
                "Vis/MinInliers":
                "8"
            }],
            remappings=[
                ("left/image_rect",
                 ConditionalText(
                     ''.join([
                         LaunchConfiguration('namespace').perform(context),
                         LaunchConfiguration('stereo_namespace').perform(
                             context),
                         LaunchConfiguration('left_image_topic').perform(
                             context), "_relay"
                     ]), ''.join([
                         LaunchConfiguration('namespace').perform(context),
                         LaunchConfiguration('stereo_namespace').perform(
                             context),
                         LaunchConfiguration('left_image_topic').perform(
                             context)
                     ]),
                     LaunchConfiguration('compressed').perform(context))),
                ("right/image_rect",
                 ConditionalText(
                     ''.join([
                         LaunchConfiguration('namespace').perform(context),
                         LaunchConfiguration('stereo_namespace').perform(
                             context),
                         LaunchConfiguration('right_image_topic').perform(
                             context), "_relay"
                     ]), ''.join([
                         LaunchConfiguration('namespace').perform(context),
                         LaunchConfiguration('stereo_namespace').perform(
                             context),
                         LaunchConfiguration('right_image_topic').perform(
                             context)
                     ]),
                     LaunchConfiguration('compressed').perform(context))),
                ("left/camera_info", [
                    LaunchConfiguration('namespace').perform(context),
                    LaunchConfiguration('stereo_namespace').perform(context),
                    LaunchConfiguration('left_camera_info_topic').perform(
                        context)
                ]),
                ("right/camera_info", [
                    LaunchConfiguration('namespace').perform(context),
                    LaunchConfiguration('stereo_namespace').perform(context),
                    LaunchConfiguration('right_camera_info_topic').perform(
                        context)
                ]),
                ("rgbd_image",
                 ConditionalText(
                     ''.join([
                         LaunchConfiguration('namespace').perform(context),
                         LaunchConfiguration('stereo_namespace').perform(
                             context),
                         LaunchConfiguration('rgbd_topic').perform(context)
                     ]), ''.join([
                         LaunchConfiguration('namespace').perform(context),
                         LaunchConfiguration('stereo_namespace').perform(
                             context),
                         LaunchConfiguration('rgbd_topic').perform(context),
                         "_relay"
                     ]),
                     LaunchConfiguration('rgbd_sync').perform(context))),
                ("odom", LaunchConfiguration('odom_topic')),
                ("imu", LaunchConfiguration('imu_topic'))
            ],
            arguments=[
                LaunchConfiguration("args"),
                LaunchConfiguration("odom_args"),
                 '--ros-args', '--log-level', LaunchConfiguration("log_level")
            ],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),

        # ICP odometry
        Node(package='rtabmap_ros',
             executable='icp_odometry',
             output="screen",
             condition=IfCondition(LaunchConfiguration('icp_odometry')),
             parameters=[{
                 "frame_id":
                 LaunchConfiguration('frame_id'),
                 "odom_frame_id":
                 LaunchConfiguration('vo_frame_id'),
                 "publish_tf":
                 LaunchConfiguration('publish_tf_odom'),
                 "ground_truth_frame_id":
                 LaunchConfiguration('ground_truth_frame_id').perform(context),
                 "ground_truth_base_frame_id":
                 LaunchConfiguration('ground_truth_base_frame_id').perform(
                     context),
                 "wait_for_transform":
                 LaunchConfiguration('wait_for_transform'),
                 "wait_imu_to_init":
                 LaunchConfiguration('wait_imu_to_init'),
                 "approx_sync":
                 LaunchConfiguration('approx_sync'),
                 "config_path":
                 LaunchConfiguration('cfg').perform(context),
                 "queue_size":
                 LaunchConfiguration('queue_size'),
                 "qos":
                 LaunchConfiguration('qos_image'),
                 "qos_imu":
                 LaunchConfiguration('qos_imu'),
                 "guess_frame_id":
                 LaunchConfiguration('odom_guess_frame_id').perform(context),
                 "guess_min_translation":
                 LaunchConfiguration('odom_guess_min_translation'),
                 "guess_min_rotation":
                 LaunchConfiguration('odom_guess_min_rotation')
             }],
             remappings=[("scan", LaunchConfiguration('scan_topic')),
                         ("scan_cloud",
                          LaunchConfiguration('scan_cloud_topic')),
                         ("odom", LaunchConfiguration('odom_topic')),
                         ("imu", LaunchConfiguration('imu_topic'))],
             arguments=[
                 LaunchConfiguration("args"),
                 LaunchConfiguration("odom_args")
             ],
             prefix=LaunchConfiguration('launch_prefix'),
             namespace=LaunchConfiguration('namespace')),
    ]


def generate_launch_description():

    config_rviz = os.path.join(get_package_share_directory('rtabmap_ros'),
                               'launch', 'config', 'rgbd.rviz')

    return LaunchDescription([

        # Arguments
        DeclareLaunchArgument(
            'stereo',
            default_value='true',
            description='Use stereo input instead of RGB-D.'),
        DeclareLaunchArgument('localization',
                              default_value='false',
                              description='Launch in localization mode.'),
        DeclareLaunchArgument('rtabmapviz',
                              default_value='true',
                              description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',
                              default_value='false',
                              description='Launch RVIZ (optional).'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Config files
        DeclareLaunchArgument(
            'cfg',
            default_value='',
            description=
            'To change RTAB-Map\'s parameters, set the path of config file (*.ini) generated by the standalone app.'
        ),
        DeclareLaunchArgument('gui_cfg',
                              default_value='~/.ros/rtabmap_gui.ini',
                              description='Configuration path of rtabmapviz.'),
        DeclareLaunchArgument('rviz_cfg',
                              default_value=config_rviz,
                              description='Configuration path of rviz2.'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='camera_link',
            description=
            'Fixed frame id of the robot (base frame), you may set "base_link" or "base_footprint" if they are published. For camera-only config, this could be "camera_link".'
        ),
        DeclareLaunchArgument(
            'odom_frame_id',
            default_value='',
            description=
            'If set, TF is used to get odometry instead of the topic.'),
        DeclareLaunchArgument('map_frame_id',
                              default_value='map',
                              description='Output map frame id (TF).'),
        DeclareLaunchArgument(
            'publish_tf_map',
            default_value='true',
            description='Publish TF between map and odomerty.'),
        DeclareLaunchArgument('namespace', default_value='/r0',
                              description=''),
        DeclareLaunchArgument('log_level', default_value='info',
                              description=''),
        DeclareLaunchArgument('queue_size', default_value='10',
                              description=''),
        DeclareLaunchArgument(
            'qos',
            default_value='2',
            description=
            'General QoS used for sensor input data: 0=system default, 1=Reliable, 2=Best Effort.'
        ),
        DeclareLaunchArgument('wait_for_transform',
                              default_value='0.2',
                              description=''),
        DeclareLaunchArgument(
            'rtabmap_args',
            default_value='',
            description='Backward compatibility, use "args" instead.'),
        DeclareLaunchArgument(
            'launch_prefix',
            default_value='',
            description=
            'For debugging purpose, it fills prefix tag of the nodes, e.g., "xterm -e gdb -ex run --args"'
        ),
        DeclareLaunchArgument(
            'output',
            default_value='screen',
            description='Control node output (screen or log).'),
        DeclareLaunchArgument('ground_truth_frame_id',
                              default_value='',
                              description='e.g., "world"'),
        DeclareLaunchArgument(
            'ground_truth_base_frame_id',
            default_value='',
            description=
            'e.g., "tracker", a fake frame matching the frame "frame_id" (but on different TF tree)'
        ),
        DeclareLaunchArgument(
            'approx_sync',
            default_value='false',
            description=
            'If timestamps of the input topics should be synchronized using approximate or exact time policy.'
        ),

        # RGB-D related topics
        DeclareLaunchArgument('rgb_topic',
                              default_value=[
                                  LaunchConfiguration('namespace'),
                                  '/camera/rgb/image_rect_color'
                              ],
                              description=''),
        DeclareLaunchArgument('depth_topic',
                              default_value=[
                                  LaunchConfiguration('namespace'),
                                  '/camera/depth_registered/image_raw'
                              ],
                              description=''),
        DeclareLaunchArgument('camera_info_topic',
                              default_value=[
                                  LaunchConfiguration('namespace'),
                                  '/camera/rgb/camera_info'
                              ],
                              description=''),

        # Stereo related topics
        DeclareLaunchArgument('stereo_namespace',
                              default_value='/stereo_camera',
                              description=''),
        DeclareLaunchArgument('left_image_topic',
                              default_value='/left/image_rect_color',
                              description=''),
        DeclareLaunchArgument('right_image_topic',
                              default_value='/right/image_rect_color'),
        DeclareLaunchArgument('left_camera_info_topic',
                              default_value='/left/camera_info',
                              description=''),
        DeclareLaunchArgument('right_camera_info_topic',
                              default_value='/right/camera_info',
                              description=''),

        # Use Pre-sync RGBDImage format
        DeclareLaunchArgument(
            'rgbd_sync',
            default_value='false',
            description='Pre-sync rgb_topic, depth_topic, camera_info_topic.'),
        DeclareLaunchArgument('approx_rgbd_sync',
                              default_value='true',
                              description='false=exact synchronization.'),
        DeclareLaunchArgument(
            'subscribe_rgbd',
            default_value=LaunchConfiguration('rgbd_sync'),
            description=
            'Already synchronized RGB-D related topic, e.g., with rtabmap_ros/rgbd_sync nodelet.'
        ),
        DeclareLaunchArgument('rgbd_topic',
                              default_value='rgbd_image',
                              description=''),
        DeclareLaunchArgument('depth_scale',
                              default_value='1.0',
                              description=''),

        # Image topic compression
        DeclareLaunchArgument(
            'compressed',
            default_value='false',
            description='If you want to subscribe to compressed image topics'),
        DeclareLaunchArgument(
            'rgb_image_transport',
            default_value='compressed',
            description=
            'Common types: compressed, theora (see "rosrun image_transport list_transports")'
        ),
        DeclareLaunchArgument(
            'depth_image_transport',
            default_value='compressedDepth',
            description=
            'Depth compatible types: compressedDepth (see "rosrun image_transport list_transports")'
        ),

        # LiDAR
        DeclareLaunchArgument('subscribe_scan',
                              default_value='false',
                              description=''),
        DeclareLaunchArgument('scan_topic',
                              default_value='/scan',
                              description=''),
        DeclareLaunchArgument('subscribe_scan_cloud',
                              default_value='false',
                              description=''),
        DeclareLaunchArgument('scan_cloud_topic',
                              default_value='/scan_cloud',
                              description=''),
        DeclareLaunchArgument('scan_normal_k',
                              default_value='0',
                              description=''),

        # Odometry
        DeclareLaunchArgument(
            'visual_odometry',
            default_value='true',
            description='Launch rtabmap visual odometry node.'),
        DeclareLaunchArgument('icp_odometry',
                              default_value='false',
                              description='Launch rtabmap icp odometry node.'),
        DeclareLaunchArgument('odom_topic',
                              default_value='odom',
                              description='Odometry topic name.'),
        DeclareLaunchArgument(
            'vo_frame_id',
            default_value=LaunchConfiguration('odom_topic'),
            description='Visual/Icp odometry frame ID for TF.'),
        DeclareLaunchArgument('publish_tf_odom',
                              default_value='true',
                              description=''),
        DeclareLaunchArgument(
            'odom_tf_angular_variance',
            default_value='0.01',
            description=
            'If TF is used to get odometry, this is the default angular variance'
        ),
        DeclareLaunchArgument(
            'odom_tf_linear_variance',
            default_value='0.001',
            description=
            'If TF is used to get odometry, this is the default linear variance'
        ),
        DeclareLaunchArgument(
            'odom_args',
            default_value='',
            description=
            'More arguments for odometry (overwrite same parameters in rtabmap_args).'
        ),
        DeclareLaunchArgument('odom_sensor_sync',
                              default_value='false',
                              description=''),
        DeclareLaunchArgument('odom_guess_frame_id',
                              default_value='',
                              description=''),
        DeclareLaunchArgument('odom_guess_min_translation',
                              default_value='0.0',
                              description=''),
        DeclareLaunchArgument('odom_guess_min_rotation',
                              default_value='0.0',
                              description=''),

        # imu
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description=
            'Used with VIO approaches and for SLAM graph optimization (gravity constraints).'
        ),
        DeclareLaunchArgument('wait_imu_to_init',
                              default_value='false',
                              description=''),

        # User Data
        DeclareLaunchArgument(
            'subscribe_user_data',
            default_value='false',
            description='User data synchronized subscription.'),
        DeclareLaunchArgument('user_data_topic',
                              default_value='/user_data',
                              description=''),
        DeclareLaunchArgument(
            'user_data_async_topic',
            default_value='/user_data_async',
            description=
            'User data async subscription (rate should be lower than map update rate).'
        ),

        #GPS
        DeclareLaunchArgument(
            'gps_topic',
            default_value='/gps/fix',
            description=
            'GPS async subscription. This is used for SLAM graph optimization and loop closure candidates selection.'
        ),

        # Tag/Landmark
        DeclareLaunchArgument(
            'tag_topic',
            default_value='/tag_detections',
            description=
            'AprilTag topic async subscription. This is used for SLAM graph optimization and loop closure detection. Landmark poses are also published accordingly to current optimized map.'
        ),
        DeclareLaunchArgument('tag_linear_variance',
                              default_value='0.0001',
                              description=''),
        DeclareLaunchArgument(
            'tag_angular_variance',
            default_value='9999.0',
            description=
            '>=9999 means rotation is ignored in optimization, when rotation estimation of the tag is not reliable or not computed.'
        ),
        DeclareLaunchArgument(
            'fiducial_topic',
            default_value='/fiducial_transforms',
            description=
            'aruco_detect async subscription, use tag_linear_variance and tag_angular_variance to set covariance.'
        ),
        DeclareLaunchArgument('database_path',
                              default_value='~/.ros/',
                              description='Where is the map saved/loaded.'),
        OpaqueFunction(function=launch_setup)
    ])
