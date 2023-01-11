import cslam.utils.point_cloud2 as pc2_utils
from geometry_msgs.msg import Transform
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField

import numpy as np
import teaserpp_python
import open3d
from scipy.spatial import cKDTree
import rclpy

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]

# Partially adapted from https://github.com/MIT-SPARK/TEASER-plusplus/tree/master/examples/teaser_python_fpfh_icp


def pcd2xyz(pcd):
    return np.asarray(pcd.points).T


def extract_fpfh(pcd, voxel_size):
    radius_normal = voxel_size * 2
    pcd.estimate_normals(
        open3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal,
                                                max_nn=30))

    radius_feature = voxel_size * 5
    fpfh = open3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        open3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature,
                                                max_nn=100))
    return np.array(fpfh.data).T


def find_knn_cpu(feat0, feat1, knn=1, return_distance=False):
    feat1tree = cKDTree(feat1)
    dists, nn_inds = feat1tree.query(feat0, k=knn, workers=-1)
    if return_distance:
        return nn_inds, dists
    else:
        return nn_inds


def find_correspondences(feats0, feats1, mutual_filter=True):
    nns01 = find_knn_cpu(feats0, feats1, knn=1, return_distance=False)
    corres01_idx0 = np.arange(len(nns01))
    corres01_idx1 = nns01

    if not mutual_filter:
        return corres01_idx0, corres01_idx1

    nns10 = find_knn_cpu(feats1, feats0, knn=1, return_distance=False)
    corres10_idx1 = np.arange(len(nns10))
    corres10_idx0 = nns10

    mutual_filter = (corres10_idx0[corres01_idx1] == corres01_idx0)
    corres_idx0 = corres01_idx0[mutual_filter]
    corres_idx1 = corres01_idx1[mutual_filter]

    return corres_idx0, corres_idx1


def get_teaser_solver(noise_bound):
    solver_params = teaserpp_python.RobustRegistrationSolver.Params()
    solver_params.cbar2 = 1.0
    solver_params.noise_bound = noise_bound
    solver_params.estimate_scaling = False
    solver_params.inlier_selection_mode = \
        teaserpp_python.RobustRegistrationSolver.INLIER_SELECTION_MODE.PMC_EXACT
    solver_params.rotation_tim_graph = \
        teaserpp_python.RobustRegistrationSolver.INLIER_GRAPH_FORMULATION.CHAIN
    solver_params.rotation_estimation_algorithm = \
        teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
    solver_params.rotation_gnc_factor = 1.4
    solver_params.rotation_max_iterations = 10000
    solver_params.rotation_cost_threshold = 1e-16
    solver = teaserpp_python.RobustRegistrationSolver(solver_params)
    return solver


def Rt2T(R, t):
    T = np.identity(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def downsample(points, voxel_size):
    open3d_cloud = open3d.geometry.PointCloud()
    open3d_cloud.points = open3d.utility.Vector3dVector(points)
    return open3d_cloud.voxel_down_sample(voxel_size=voxel_size)


def solve_teaser(src, dst, voxel_size, min_inliers):
    src_feats = extract_fpfh(src, voxel_size)
    dst_feats = extract_fpfh(dst, voxel_size)

    corrs_src, corrs_dst = find_correspondences(src_feats,
                                                dst_feats,
                                                mutual_filter=True)

    src_xyz = pcd2xyz(src)  # np array of size 3 by N
    dst_xyz = pcd2xyz(dst)  # np array of size 3 by M
    src_corr = src_xyz[:, corrs_src]  # np array of size 3 by num_corrs
    dst_corr = dst_xyz[:, corrs_dst]  # np array of size 3 by num_corrs

    solver = get_teaser_solver(voxel_size)
    solver.solve(src_corr, dst_corr)

    solution = solver.getSolution()

    valid = len(solver.getInlierMaxClique()) > min_inliers

    if valid:
        # ICP refinement
        T_teaser = Rt2T(solution.rotation, solution.translation)
        icp_sol = open3d.pipelines.registration.registration_icp(
            src, dst, voxel_size, T_teaser,
            open3d.pipelines.registration.TransformationEstimationPointToPoint(
            ),
            open3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=100))
        T_icp = icp_sol.transformation
        solution.translation = T_icp[:3, 3]
        solution.rotation = T_icp[:3, :3]
    else:
        rclpy.logging.get_logger('cslam').info(
            'Failed to compute loop closure. Number of inliers ( {} / {} )'.
            format(len(solver.getInlierMaxClique()), min_inliers))
    return valid, solution.translation, solution.rotation


def to_transform_msg(translation, rotation):
    T = Transform()
    T.translation.x = translation[0]
    T.translation.y = translation[1]
    T.translation.z = translation[2]
    rotation_matrix = R.from_matrix(np.array(rotation))
    q = rotation_matrix.as_quat()
    T.rotation.x = q[0]
    T.rotation.y = q[1]
    T.rotation.z = q[2]
    T.rotation.w = q[3]
    return T


def open3d_to_ros(open3d_cloud):
    header = Header()
    fields = FIELDS_XYZ
    points = np.asarray(open3d_cloud.points)
    return pc2_utils.create_cloud(header, fields, points)


def ros_to_open3d(msg):
    points = ros_pointcloud_to_points(msg)
    open3d_cloud = open3d.geometry.PointCloud()
    open3d_cloud.points = open3d.utility.Vector3dVector(points)
    return open3d_cloud


def ros_pointcloud_to_points(pc_msg):
    return pc2_utils.read_points_numpy_filtered(pc_msg)[:, :3]


def downsample_ros_pointcloud(pc_msg, voxel_size):
    points = ros_pointcloud_to_points(pc_msg)
    return downsample(points, voxel_size)

def compute_transform(src, dst, voxel_size, min_inliers):
    """Computes a 3D transform between 2 point clouds using TEASER++.

    Be careful: TEASER++ computes the transform from dst to src.

    Args:
        src (Open3D point cloud): pointcloud from
        dst (Open3D point cloud): pointcloud to
        voxel_size (int): Voxel size
        min_inliers (int): Minimum number of inlier points correspondance to consider the registration a success

    Returns:
        (Transform, bool): transform and success flag
    """
    valid, translation, rotation = solve_teaser(src, dst, voxel_size,
                                                min_inliers)
    success = valid
    transform = to_transform_msg(translation, rotation)
    return transform, success