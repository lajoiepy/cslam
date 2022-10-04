import numpy as np
from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from cslam_common_interfaces.msg import KeyframeOdom
from cslam_loop_detection_interfaces.msg import LocalDescriptorsRequest, LocalPointCloudDescriptors, InterRobotLoopClosure, IntraRobotLoopClosure, LocalKeyframeMatch
import cslam.lidar_pr.icp_utils as icp_utils

class LidarHandler:
    def __init__(self, node, params):
        self.node = node
        self.params = params

        tss = TimeSynchronizer( Subscriber(self.node, self.params["frontend.pointcloud_topic"], PointCloud2),
                                Subscriber(self.node, self.params["frontend.odom_topic"], Odometry) )
        tss.registerCallback(self.lidar_callback)

        self.keyframe_odom_publisher = self.node.create_publisher(KeyframeOdom, "keyframe_odom", 100)

        self.send_local_descriptors_subscriber = self.node.create_subscription(LocalDescriptorsRequest,
                                                                            "local_descriptors_request", self.send_local_descriptors_request, 100)

        self.local_keyframe_match_subscriber = self.node.create_subscription(LocalKeyframeMatch,
                                                                            "local_keyframe_match", self.receive_local_keyframe_match, 100)
        
        self.pointcloud_descriptors_publisher = self.node.create_publisher(LocalPointCloudDescriptors, "/local_descriptors", 100)
        
        self.pointcloud_descriptors_subscriber = self.node.create_subscription(LocalPointCloudDescriptors, "/local_descriptors", self.receive_local_descriptors, 100)
        
        self.intra_robot_loop_closure_publisher = self.node.create_publisher(IntraRobotLoopClosure, "intra_robot_loop_closure", 100)

        self.inter_robot_loop_closure_publisher = self.node.create_publisher(InterRobotLoopClosure, "/inter_robot_loop_closure", 100)

        period_ms = self.params["frontend.map_manager_process_period_ms"]
        self.processing_timer = self.node.create_timer(float(period_ms)/1000, self.process_new_sensor_data)

        self.received_data = []
        self.local_descriptors_map = {}
        self.nb_local_keyframes = 0
        self.previous_keyframe = None

    def lidar_callback(self, pc_msg, odom_msg):
        self.received_data.append((pc_msg, odom_msg))

    def send_local_descriptors_request(self, request):
        out_msg = LocalPointCloudDescriptors()
        out_msg.data = icp_utils.open3d_to_ros(self.local_descriptors_map[request.image_id])
        out_msg.image_id = request.image_id
        out_msg.robot_id = self.params["robot_id"]
        out_msg.matches_robot_id = request.matches_robot_id
        out_msg.matches_image_id = request.matches_image_id

        self.pointcloud_descriptors_publisher.publish(out_msg)
    
    def receive_local_descriptors(self, msg):
        frame_ids = []
        for i in range(len(msg.matches_robot_id)):
            if msg.matches_robot_id[i] == self.params["robot_id"]:
                frame_ids.append(msg.matches_image_id[i])
        for frame_id in frame_ids:
            pc = self.local_descriptors_map[frame_id]
            transform, success = icp_utils.compute_transform(pc, icp_utils.ros_to_open3d(msg.data), self.params["frontend.voxel_size"])
            out_msg = InterRobotLoopClosure()
            out_msg.robot0_id = self.params["robot_id"]
            out_msg.robot0_image_id = frame_id
            out_msg.robot1_id = msg.robot_id
            out_msg.robot1_image_id = msg.image_id
            if success:
                out_msg.success = True
                out_msg.transform = transform
            else:
                out_msg.success = False
            self.inter_robot_loop_closure_publisher.publish(out_msg)

    def receive_local_keyframe_match(self, msg):
        pc0 = self.local_descriptors_map[msg.keyframe0_id]
        pc1 = self.local_descriptors_map[msg.keyframe1_id]
        transform, success = icp_utils.compute_transform(pc0, pc1, self.params["frontend.voxel_size"])
        out_msg = IntraRobotLoopClosure()
        out_msg.keyframe0_id = msg.keyframe0_id
        out_msg.keyframe1_id = msg.keyframe1_id
        if success:
            out_msg.success = True
            out_msg.transform = transform
        else:
            out_msg.success = False
        self.intra_robot_loop_closure_publisher.publish(out_msg)

    def generate_new_keyframe(self, msg):
        # TODO: Perform overlap check, instead of consering each frame as a keyframe
        return True 

    def process_new_sensor_data(self):
        if len(self.received_data) > 0:
            data = self.received_data[0]
            self.received_data.pop(0)
            if self.generate_new_keyframe(data):
                self.local_descriptors_map[self.nb_local_keyframes] = icp_utils.downsample_ros_pointcloud(data[0], self.params["frontend.voxel_size"])
                msg_odom = KeyframeOdom()
                msg_odom.id = self.nb_local_keyframes
                msg_odom.odom = data[1]
                self.keyframe_odom_publisher.publish(msg_odom)
                self.nb_local_keyframes = self.nb_local_keyframes + 1
