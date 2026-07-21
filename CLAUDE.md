# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
colcon build
# To build only one package:
colcon build --packages-select basic
colcon build --packages-select super_lio
```

Source the workspace before running:
```bash
source install/setup.bash
```

**Requirements**: C++20, ROS2 Humble/Iron/Jazzy, Eigen3, PCL, glog, TBB, livox_ros_driver2. Install system deps: `sudo apt install libgoogle-glog-dev libtbb-dev`.

## Run

```bash
ros2 launch super_lio Livox_mid360.py          # Standard SLAM
ros2 launch super_lio relocation.py            # Relocalization mode
ros2 launch super_lio Livox_mid360.py rviz:=false  # Headless (no RViz)
```

Config files live in `src/super_lio/config/` (one YAML per sensor/dataset). Launch files in `src/super_lio/launch/` (one `.py` per config).

## Architecture

Two ROS2 packages: `basic` (shared math/utility library) and `super_lio` (the LIO system). `super_lio` depends on `basic`.

### `basic` — Foundation library

- **`basic/alias.h`**: Eigen type aliases — critical: `BASIC::scalar = float` (single precision), not double. All Eigen matrices/vectors used across the codebase are `float` by default (V3 = Vector3f, M3 = Matrix3f, etc.). Double-precision variants are suffixed with `d` (V3d, M3d).
- **`basic/Manifold.h`**: `SO3`, `SE3` Lie group classes with Exp/Log, adjoints, update functions. Also contains free functions `SKEW`, `RightJacobianSO3`, `A_matrix` (left Jacobian).
- **`basic/ds.h`**: Data structs — `RobotState`, `NavState`, `Pose_t`, `ObsData`.
- **`basic/logs.h`**: glog-based colored logging (`LOG(INFO) << GREEN << ... << RESET`).
- **`basic/buffer/`**: Template buffer classes (`RingBuffer`, `LatestOnlyBuffer`, `MultiSourceLatestBuffer`).

### `super_lio` — Main LIO system

#### Core algorithm (`src/lio/`, `include/lio/`)

- **`SuperLIO`** (`super_lio.h/.cpp`): Base class implementing the core pipeline as a state machine:
  1. `stateWaitKFInit` — static IMU gravity alignment → initialize `ESKF`
  2. `stateWaitMapInit` — accumulate initial scans into `OctVoxMap` (~20 frames)
  3. `stateProcess` — main loop: `Propagation_Undistort → DownSample → Observe → UpdateMap → Output → caceData`

  The state transitions via function pointers (`StateFn state_fn_`). `process()` is called on a 2ms ROS2 wall timer; it calls `sync_measure()` on ROSWrapper and dispatches to the current state function.

- **`SuperLIOReLoc`** (`super_lio_reloc.h/.cpp`): Inherits `SuperLIO`, overrides `init()`, `kf_init()`, `map_init()`, `UpdateMap()`, `Output()` for relocalization against a pre-built map (no map insertion, only localization).

- **`ESKF`** (`ESKF.h/.cpp`): Error-State Kalman Filter. 18-DOF state: [R(3), p(3), v(3), bg(3), ba(3), g(3)]. The observation update uses iterative IEKF: `UpdateObserve()` takes a lambda `ObsFunc` that computes `H^T V^{-1} H` and `H^T V^{-1} r`. The observation model (point-to-plane residuals + covariance) is built inside `SuperLIO::Observe()`, decoupling it from the filter. `Predict()` propagates with IMU using right-Jacobian of SO(3).

- **`params.h/.cpp`**: All algorithm parameters as `extern` globals with `g_` prefix (e.g., `g_lidar_imu`, `g_ivox_resolution`, `g_save_map`). Loaded from ROS2 YAML parameters by `LoadParamFromRos()` in ROSWrapper.

- **`common/ds.h`**: Algorithm-side data structures — `SysState`, `NavState`, `DynamicState`, `Pose_t`, `IMUData`, `LidarData`, `MeasureGroup`.

#### Mapping (`include/OctVoxMap/`)

- **`OctVoxMap`**: Spatial hash map using Tessil's `robin_map` for voxel storage. Fast KNN queries via `getTopK()`.
- **`KNNHeap<5>`**: Fixed-size 5-nearest-neighbor heap for plane fitting in `Observe()`.
- **`VoxelGridFilter`**: Voxel grid with per-cell closest-point selection.

#### ROS interface (`src/ros/`, `include/ros/`)

- **`ROSWrapper`** (`ROSWrapper.h/.cpp`): Inherits `rclcpp::Node`. Handles all ROS communication — IMU/LiDAR subscriptions, time synchronization (`sync_measure()`), odometry/path publishing, TF. Uses a separate callback group for sensor subscriptions.

#### Apps (`src/apps/`)

- `super_lio_node.cpp` — standard SLAM
- `relocation_node.cpp` — relocalization

### Key design patterns

- **ROS/algorithm separation**: `ROSWrapper` handles all ROS concerns; `SuperLIO` is pure algorithm. ROSWrapper is injected via `setROSWrapper()`. This is why the ros1 branch can share the same core algorithm.
- **TBB parallelism**: `tbb::parallel_for` with `blocked_range` is used for per-point operations in `Propagation_Undistort()` and `Observe()`. `Observe()` uses `tbb::enumerable_thread_specific` for per-thread accumulator reduction.
- **Global parameters**: ROS2 params are loaded into `g_*` globals at startup. No parameter server polling at runtime.
- **Hybrid residual**: `ResidualType` enum (PROB/P2P/MIX) selects between probabilistic, point-to-plane, or hybrid observation residuals in `Observe()`.
- **Scalar precision**: The entire system defaults to `float` (`BASIC::scalar = float`). Intermediate computations in `Observe()` and `ESKF` use `double` casts where numerical stability matters.
- **Multi-LiDAR support**: Point types for Livox, Velodyne, Hesai, NCLT, Ouster are defined in `basic/alias.h`. The `LID_TYPE` enum maps to specific scan pattern parameters.

### Cross-branch notes

The `ros1` branch shares the core algorithm. The ROS2 branch (`ros2`) refactored the ROS interface into `ROSWrapper` while keeping `SuperLIO` algorithm code largely identical.

## Git conventions

Commits are in imperative mood.
