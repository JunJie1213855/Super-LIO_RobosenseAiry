# SuperLIO SLAM

This ROS 2 package contains a local copy of Super-LIO, its `basic` code, and STD. The online pipeline is:

1. Super-LIO processes LiDAR and IMU data and publishes selected keyframes after its normal observation update.
2. The backend extracts STD descriptors and searches earlier keyframes for a loop candidate.
3. ICP aligns the current keyframe with the candidate's local submap. Convergence, overlap, fitness, and correction limits gate the result.
4. Accepted ICP relative poses become robust loop factors in an iSAM2 pose graph. The backend publishes the corrected trajectory and map.

ICP supplies a pose-graph observation; it does not alter the front-end ESKF state. The front end continues publishing `world → imu`, and the backend publishes `map → world`, so TF consumers can obtain the corrected `map → imu` pose.

## Build and run

Source ROS 2 Humble and the workspace containing `livox_ros_driver2`, then build this package:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select super_lio_slam --symlink-install
source install/setup.bash
ros2 launch super_lio_slam slam.launch.py
```

The SLAM launch defaults to `config/livox_360.yaml` for the sensor/front end and `config/slam.yaml` for keyframes and the backend. Select another copied sensor configuration with `sensor_config:=/absolute/path/to/config.yaml`. Add `rviz:=true` to start RViz.

For recorded data, use one time source for both nodes and RViz. Start the launch first, then play the bag in another terminal:

```bash
ros2 launch super_lio_slam slam.launch.py use_sim_time:=true rviz:=true
ros2 bag play /home/ros/dataset/mid360/jungle_challenge --clock
```

Restart the launch before replaying the same bag from its beginning; that also clears the pose graph and TF history. Live sensors should retain the default `use_sim_time:=false`.

The Robosense Airy `mapping` bag uses device-uptime stamps (about 11860 s) in LiDAR and IMU messages, while its `/clock` uses the recording epoch. Use the Robosense launch and `--clock` together:

```bash
ros2 launch super_lio_slam robo_slam.launch.py rviz:=true use_sim_time:=true
ros2 bag play /home/ros/dataset/robosense/mapping --clock
```

`config/robosense_airy.yaml` enables `lio.ros.align_sensor_time_to_ros_clock`. It waits for the simulated ROS clock, then computes one offset for published odometry, clouds, keyframes, and TF; the LIO estimator still uses the original shared device time for LiDAR/IMU synchronization. Other sensor configurations leave this opt-in setting disabled.

For a Unitree UniLiDAR L1 publishing `/unilidar/cloud` and `/unilidar/imu`, use:

```bash
ros2 launch super_lio_slam unitree_slam.launch.py rviz:=true use_sim_time:=true
ros2 bag play /home/ros/dataset/unitree/L1 --clock
```

`config/unitree_l1.yaml` selects LiDAR type `9`. The adapter reads the
`PointCloud2` fields `x`, `y`, `z`, `intensity`, `ring`, and float32 `time`;
`time` is a per-point offset from the message header in seconds (adjust
`lio.sensor.timestamp_unit` if another Unitree driver uses milliseconds,
microseconds, or nanoseconds). The same adapter can be used for an L2 driver
with that field layout. The configured LiDAR-to-IMU extrinsic is a starting
value from the local Point-LIO L1 profile and should be calibrated for the
physical sensor.

The copied single-node launch files continue to run Super-LIO without the loop backend. `ros2 launch super_lio_slam std_extract.launch.py` runs the copied STD descriptor extraction utility independently.

## Data contract

`/lio_slam/keyframe` uses `super_lio_slam/msg/KeyFrame`. `odom_pose` is the uncorrected end-of-scan IMU pose in `world`; `cloud` is deskewed and expressed in that end-of-scan IMU frame. The keyframe ID and timestamp identify the corresponding pose and cloud. The backend publishes:

| Topic | Type | Frame |
| --- | --- | --- |
| `/lio_slam/odom` | `nav_msgs/Odometry` | `map → imu` |
| `/lio_slam/path` | `nav_msgs/Path` | `map` |
| `/lio_slam/map` | `sensor_msgs/PointCloud2` | `map` |
| `/lio_slam/loop_markers` | `visualization_msgs/MarkerArray` | `map` |
| `/lio_slam/std_triangles` | `visualization_msgs/MarkerArray` | `map` |

The SLAM RViz view draws the latest keyframe's STD triangles in cyan with yellow vertices. Set `std_visualization_max_triangles` in `config/slam.yaml` to limit clutter, or set it to `0` to hide them. The markers use corrected `map` coordinates and replace the previous keyframe's triangles.

`config/slam.yaml` contains all keyframe, STD, ICP, graph, and map parameters. `icp_fitness_threshold` is the mean squared correspondence error in square metres. `map_max_keyframes` limits the number of keyframes used when rebuilding a published map; set it to `0` for the full accumulated trajectory. `map_max_points_per_keyframe` bounds map publication cost independently of the stored registration clouds.

For an initial check, inspect `/lio_slam/keyframe`, `/lio_slam/path`, and the `map → world → imu` TF chain while replaying a bag that revisits a place. A candidate becomes a graph factor only after the ICP gates pass; compare the corrected path and map with the front-end path after a completed loop.

The included ROS smoke test publishes five keyframes to the actual backend process. It checks a stationary revisit, a translated and rotated revisit, and an ICP rejection:

```bash
python3 src/SuperLIO_SLAM/test/online_loop_smoke.py --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node
python3 src/SuperLIO_SLAM/test/online_loop_smoke.py --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node --moving
python3 src/SuperLIO_SLAM/test/online_loop_smoke.py --backend install/super_lio_slam/lib/super_lio_slam/loop_backend_node --reject-icp
```

Run these from the workspace root after sourcing `install/setup.bash`. `--pcd /absolute/path/to/binary_float32.pcd` tests with an existing XYZI scene; it repeats the same scene under known poses rather than replaying a sensor bag.
