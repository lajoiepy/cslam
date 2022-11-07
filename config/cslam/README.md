## C-SLAM Parameters

`left_image_topic, right_image_topic, left_camera_info_topic, right_camera_info_topic, etc.`: ROS 2 topic names on which the sensor publish its data and calibration.

`odom_topic`: Topic name for input odometry estimates

`sensor_base_frame_id`: ROS 2 TF base frame of the sensor

### Front-End:

`inter_robot_loop_closure_budget`: Maximum number of loop closures that can be geometrically verified every detection periods.

`inter_robot_detection_period_sec`: Period (in seconds) for inter-robot loop closure detection.

`max_queue_size`: Maximum number of keyframe to store in buffer for processing.

`similarity_threshold`: Minimum similarity value required to consider a potential match.

`global_descriptors_topic`: Topic name for global descriptors

`global_descriptor_technique`: Global descriptors computation technique (e.g., netvlad)

`image_crop_size`: Cropping size for embedding computation

`nn_checkpoint`: Neural network model path

`pca_checkpoint`: PCA model path

`pnp_min_inliers`: Minimum number of match inliers for transformation computation between keyframes. 

`detection_publication_period_sec`: Period of publication of global descriptors.

`detection_publication_max_elems_per_msg`: Split global descriptors messages in maximum N elements.

`enable_intra_robot_loop_closures`: Enable intra-robot loop closures computation.

`intra_loop_min_inbetween_keyframes`: Minimum number of keyframes between matches for intra-robot loop closures. Aims to avoid matching subsequent images. 

`keyframe_generation_ratio_threshold`: Generate a new keyframe if the number of inlier matches between the current frame and the previous keyframe is below the specified ratio. If set to 1.0, all frames are keyframes.

`use_vertex_cover_selection`: Use vertex cover computation to minimize communication of keyframe data.

`map_manager_process_period_ms`: Period for loop closure processing. 

### Neighbor Management:

`enable_neighbor_monitoring`: Enable monitoring of robots in communication range. If false, all robots are always considered neighbors. Set to false only when connectivity is always maintained between all the robots.

`max_heartbeat_delay_sec`: Communication delay beyond which a neighboring robot is not considered in range if it did not communicate.

`heartbeat_period_sec`: Period of heartbeat publication.

### Back-End:

`pose_graph_optimization_start_period_ms`: Period of PGO start trigger.

`pose_graph_optimization_loop_period_ms`: Period of inner loop of PGO process.

`max_waiting_time_sec`: Maximum waiting time during PGO before going back to an idle state. Prevents being lock in an intermediate state in case of disconnections or failures.

`enable_logs`: Enable logging of optimization result files.

`log_folder`: Path where to store the logged optimization files.

TODO:update