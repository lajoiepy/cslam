#!/usr/bin/env python3
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import PointCloud2, PointField, NavSatFix
from nav_msgs.msg import Odometry
from cslam_common_interfaces.msg import KeyframeOdom, KeyframePointCloud
from cslam_common_interfaces.msg import LocalDescriptorsRequest, LocalPointCloudDescriptors, InterRobotLoopClosure, IntraRobotLoopClosure, LocalKeyframeMatch
from cslam_common_interfaces.msg import VizPointCloud
import cslam.lidar_pr.icp_utils as icp_utils
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from diagnostic_msgs.msg import KeyValue

class LidarHandler: 
    """Front-End data handler for lidar data
    """
    def __init__(self, node, params):
        self.node = node
        self.params = params

        tss = ApproximateTimeSynchronizer( [ Subscriber(self.node, PointCloud2, self.params["frontend.pointcloud_topic"]),
                                  Subscriber(self.node, Odometry, self.params["frontend.odom_topic"])], 100, self.params["frontend.pointcloud_odom_approx_time_sync_s"] )
        tss.registerCallback(self.lidar_callback)

        self.keyframe_odom_publisher = self.node.create_publisher(KeyframeOdom, "cslam/keyframe_odom", 100)

        self.keyframe_pointcloud_publisher = self.node.create_publisher(KeyframePointCloud, "cslam/keyframe_data", 100)

        self.send_local_descriptors_subscriber = self.node.create_subscription(LocalDescriptorsRequest,
                                                                            "cslam/local_descriptors_request", self.send_local_descriptors_request, 100)

        self.local_keyframe_match_subscriber = self.node.create_subscription(LocalKeyframeMatch,
                                                                            "cslam/local_keyframe_match", self.receive_local_keyframe_match, 100)
        
        self.pointcloud_descriptors_publisher = self.node.create_publisher(LocalPointCloudDescriptors, "/cslam/local_descriptors", 100)
        
        self.pointcloud_descriptors_subscriber = self.node.create_subscription(LocalPointCloudDescriptors, "/cslam/local_descriptors", self.receive_local_descriptors, 100)
        
        self.intra_robot_loop_closure_publisher = self.node.create_publisher(IntraRobotLoopClosure, "cslam/intra_robot_loop_closure", 100)

        self.inter_robot_loop_closure_publisher = self.node.create_publisher(InterRobotLoopClosure, "/cslam/inter_robot_loop_closure", 100)

        self.viz_pointcloud_publisher = self.node.create_publisher(VizPointCloud, "/cslam/viz/keyframe_pointcloud", 100)

        period_ms = self.params["frontend.map_manager_process_period_ms"]
        self.processing_timer = self.node.create_timer(float(period_ms)/1000, self.process_new_sensor_data, clock=Clock())

        self.received_data = []
        self.local_descriptors_map = {}
        self.nb_local_keyframes = 0
        self.previous_keyframe = None
        self.previous_odom = None

        if self.params["evaluation.enable_logs"]:
            self.log_publisher = self.node.create_publisher(
                KeyValue, 'log_info', 100)
            self.log_local_descriptors_cumulative_communication = 0

        if self.params["evaluation.enable_gps_recording"]:
            self.gps_subscriber = self.node.create_subscription(NavSatFix, self.params["evaluation.gps_topic"], self.gps_callback, 100)
            self.gps_data = []
            self.latest_gps = NavSatFix()

    def lidar_callback(self, pc_msg, odom_msg):
        """Sensor data callback

        Args:
            pc_msg (sensor_msgs/Pointcloud2): point cloud
            odom_msg (nav_msgs/Odometry): odometry estimate
        """
        if (odom_msg.pose.covariance[0] > 1000):
            self.node.get_logger().warn("Odom tracking failed, skipping frame")
            return
        self.received_data.append((pc_msg, odom_msg))
        if self.params["evaluation.enable_gps_recording"]:
            self.gps_data.append(self.latest_gps)

    def gps_callback(self, gps_msg):
        """GPS data callback
        """
        self.latest_gps = gps_msg

    def send_local_descriptors_request(self, request):
        """Callback for local descriptors request
        """
        out_msg = LocalPointCloudDescriptors()
        out_msg.data = icp_utils.open3d_to_ros(self.local_descriptors_map[request.keyframe_id])
        out_msg.keyframe_id = request.keyframe_id
        out_msg.robot_id = self.params["robot_id"]
        out_msg.matches_robot_id = request.matches_robot_id
        out_msg.matches_keyframe_id = request.matches_keyframe_id

        self.pointcloud_descriptors_publisher.publish(out_msg)
        if self.params["evaluation.enable_logs"]:
            self.log_local_descriptors_cumulative_communication += len(out_msg.data.data)
            self.log_publisher.publish(KeyValue(key="local_descriptors_cumulative_communication", value=str(self.log_local_descriptors_cumulative_communication)))
    
    def receive_local_descriptors(self, msg):
        """Callback for local descriptors from other robots
        """
        frame_ids = []
        for i in range(len(msg.matches_robot_id)):
            if msg.matches_robot_id[i] == self.params["robot_id"]:
                frame_ids.append(msg.matches_keyframe_id[i])
        for frame_id in frame_ids:
            pc = self.local_descriptors_map[frame_id]
            transform, success = icp_utils.compute_transform(pc, icp_utils.ros_to_open3d(msg.data), self.params["frontend.voxel_size"], self.params["frontend.registration_min_inliers"])
            out_msg = InterRobotLoopClosure()
            out_msg.robot0_id = self.params["robot_id"]
            out_msg.robot0_keyframe_id = frame_id
            out_msg.robot1_id = msg.robot_id
            out_msg.robot1_keyframe_id = msg.keyframe_id
            if success:
                out_msg.success = True
                out_msg.transform = transform
            else:
                out_msg.success = False
            self.inter_robot_loop_closure_publisher.publish(out_msg)

    def receive_local_keyframe_match(self, msg):
        """Callback for local keyframe match for intra-robot loop closures
        """
        pc0 = self.local_descriptors_map[msg.keyframe0_id]
        pc1 = self.local_descriptors_map[msg.keyframe1_id]
        transform, success = icp_utils.compute_transform(pc0, pc1, self.params["frontend.voxel_size"], self.params["frontend.registration_min_inliers"])
        out_msg = IntraRobotLoopClosure()
        out_msg.keyframe0_id = msg.keyframe0_id
        out_msg.keyframe1_id = msg.keyframe1_id
        if success:
            out_msg.success = True
            out_msg.transform = transform
        else:
            out_msg.success = False
        self.intra_robot_loop_closure_publisher.publish(out_msg)

    def odom_distance_squared(self, odom0, odom1):
        """Compute the squared distance between two odometry messages
        """
        return (odom0.pose.pose.position.x - odom1.pose.pose.position.x)**2 + (odom0.pose.pose.position.y - odom1.pose.pose.position.y)**2 + (odom0.pose.pose.position.z - odom1.pose.pose.position.z)**2

    def generate_new_keyframe(self, msg):
        """Check if we should generate a new keyframe

        Args:
            msg (nav_msgs/Odometry): odometry msg

        Returns:
            bool: flag indicating if a new keyframe should be created.
        """
        if self.previous_odom is None:
            self.previous_odom = msg[1]
            return True 
        dist = self.odom_distance_squared(self.previous_odom, msg[1])
        if dist > self.params["frontend.keyframe_generation_ratio_distance"]**2:
            self.previous_odom = msg[1]
            return True
        else:
            return False

    def process_new_sensor_data(self):
        """ Process new sensor data
        """
        if len(self.received_data) > 0:
            data = self.received_data[0]
            self.received_data.pop(0)
            gps_data = None
            if self.params["evaluation.enable_gps_recording"]:
                gps = self.gps_data[0]
                self.gps_data.pop(0)
            if self.generate_new_keyframe(data):
                try:
                    self.local_descriptors_map[self.nb_local_keyframes] = icp_utils.downsample_ros_pointcloud(data[0], self.params["frontend.voxel_size"])
                except Exception as e:
                    self.local_descriptors_map[self.nb_local_keyframes] = []
                    self.node.get_logger().info("Failure to downsample point cloud to voxel size {}. {}".format(self.params["frontend.voxel_size"], e))
                    return
                # Publish pointcloud
                msg_pointcloud = KeyframePointCloud()
                msg_pointcloud.id = self.nb_local_keyframes
                msg_pointcloud.pointcloud = icp_utils.open3d_to_ros(self.local_descriptors_map[self.nb_local_keyframes])
                self.keyframe_pointcloud_publisher.publish(msg_pointcloud)
                # Publish odom
                msg_odom = KeyframeOdom()
                msg_odom.id = self.nb_local_keyframes
                msg_odom.odom = data[1]
                if self.params["evaluation.enable_gps_recording"]:
                    msg_odom.gps = gps
                self.keyframe_odom_publisher.publish(msg_odom)
                if self.params["visualization.enable"]:
                    viz_msg = VizPointCloud()
                    viz_msg.robot_id = self.params["robot_id"]
                    viz_msg.keyframe_id = self.nb_local_keyframes
                    viz_msg.pointcloud = msg_pointcloud.pointcloud
                    self.viz_pointcloud_publisher.publish(viz_msg)
                self.nb_local_keyframes = self.nb_local_keyframes + 1

if __name__ == '__main__':

    rclpy.init(args=None)
    node = Node("map_manager")
    node.declare_parameters(
            namespace='',
            parameters=[('frontend.pointcloud_topic', None),
                        ('frontend.odom_topic', None),
                        ('frontend.map_manager_process_period_ms', None),
                        ('frontend.voxel_size', None),
                        ('frontend.registration_min_inliers', None),
                        ('frontend.keyframe_generation_ratio_distance', 0.5),
                        ('frontend.pointcloud_odom_approx_time_sync_s', 0.1),
                        ('robot_id', None),           
                        ('evaluation.enable_logs', False),  
                        ('evaluation.enable_gps_recording', False), 
                        ('evaluation.gps_topic', ""),            
                        ('evaluation.gps_topic', ""),        
                        ('visualization.enable', False),
                        ])
    params = {}
    params['frontend.pointcloud_topic'] = node.get_parameter(
        'frontend.pointcloud_topic').value 
    params['frontend.odom_topic'] = node.get_parameter(
        'frontend.odom_topic').value
    params['frontend.map_manager_process_period_ms'] = node.get_parameter(
        'frontend.map_manager_process_period_ms').value
    params['frontend.voxel_size'] = node.get_parameter(
        'frontend.voxel_size').value
    params['frontend.registration_min_inliers'] = node.get_parameter(
        'frontend.registration_min_inliers').value 
    params['frontend.keyframe_generation_ratio_distance'] = node.get_parameter(
        'frontend.keyframe_generation_ratio_distance').value
    params['frontend.pointcloud_odom_approx_time_sync_s'] = node.get_parameter(
        'frontend.pointcloud_odom_approx_time_sync_s').value
    params['robot_id'] = node.get_parameter(
        'robot_id').value
    params["evaluation.enable_logs"] = node.get_parameter(
            'evaluation.enable_logs').value
    params["evaluation.enable_gps_recording"] = node.get_parameter(
            'evaluation.enable_gps_recording').value
    params["evaluation.gps_topic"] = node.get_parameter(
            'evaluation.gps_topic').value
    params["visualization.enable"] = node.get_parameter(
            'visualization.enable').value
    lidar_handler = LidarHandler(node, params)
    node.get_logger().info('Initialization done.')
    rclpy.spin(node)
    rclpy.shutdown()