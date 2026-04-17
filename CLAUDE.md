# CLAUDE.md — FAST-LIVO2

Fast, Direct LiDAR-Inertial-Visual Odometry (IEEE T-RO '24). ROS2 SLAM package fusing LiDAR, IMU, and camera for real-time 3D reconstruction.

- **Package name**: `fast_livo`
- **Executable**: `fastlivo_mapping`
- **Language**: C++17 | **Build**: ament_cmake (colcon) | **Node**: `laserMapping`
- **Active branches**: `ros2-jazzy` (development), `humble` (base)

---

## Build

Always build from the workspace root, **not** from inside this directory:

```bash
cd /gd_mapping
colcon build --symlink-install --continue-on-error
source install/setup.bash
```

The CMakeLists.txt hardcodes vikit lib paths relative to `../../install/vikit_common/lib/`, so the workspace root is required.

**Architecture-specific flags** (auto-detected):
- ARM aarch64: `-O3 -mcpu=native -mtune=native -ffast-math`
- x86-64: `-O3 -march=native -mtune=native -funroll-loops`

OpenMP and mimalloc are detected and used automatically. `MP_PROC_NUM` is capped at 4.

---

## Source Layout

```
src/
  main.cpp          — entry point; creates LIVMapper, calls run()
  LIVMapper.cpp     — main loop: sync_packages(), handleLIO(), handleVIO()
  IMU_Processing.cpp — IMU pre-integration and state propagation
  preprocess.cpp    — ring extraction, downsampling, distortion correction
  vio.cpp           — direct photometric alignment
  voxel_map.cpp     — voxel map update via ESIKF
  frame.cpp         — keyframe management
  visual_point.cpp  — 3D-2D point tracking
  utils.cpp         — misc utilities

include/
  LIVMapper.h       — main class declaration
  common_lib.h      — enums (LID_TYPE, SLAM_MODE, EKF_STATE), constants
  IMU_Processing.h
  preprocess.h
  vio.h / frame.h / visual_point.h / voxel_map.h / feature.h
```

---

## Core Data Flow

```
LiDAR callback (standard_pcl_cbk / livox_pcl_cbk)
    → preprocess.cpp: ring extraction, downsampling, undistortion
IMU callback (imu_cbk) → IMU_Processing: pre-integration, state propagation
Image callback (img_cbk)
    ↓
LIVMapper::run() — sync_packages() aligns streams by timestamp
    ↓
handleLIO() — voxel map update (ESIKF, voxel_map.cpp)
handleVIO() — photometric alignment (vio.cpp, frame.cpp, visual_point.cpp)
    ↓
Publish: odometry, colored point cloud, path, TF
```

State vector (19-DOF): rotation (SO3), position, velocity, IMU biases (acc + gyro), gravity.

---

## LiDAR Types (`include/common_lib.h`)

| Enum | Value | Sensor |
|------|-------|--------|
| `AVIA` | 1 | Livox Avia / HAP |
| `VELO16` | 2 | Velodyne 16 |
| `OUST64` | 3 | Ouster (all channel counts) |
| `L515` | 4 | Intel L515 |
| `XT32` | 5 | Hesai XT32 |
| `PANDAR128` | 6 | Pandar 128 |
| `ROBOSENSE` | 7 | RoboSense |

Set via `preprocess.lidar_type` in the main YAML.

---

## SLAM Modes

| Enum | Value | Meaning |
|------|-------|---------|
| `ONLY_LO` | 0 | LiDAR odometry only |
| `ONLY_LIO` | 1 | LiDAR-Inertial only |
| `LIVO` | 2 | Full LiDAR-Inertial-Visual |

Controlled by `img_en` / `lidar_en` in the main YAML:
- `img_en: 0, lidar_en: 1` → LIO only
- `img_en: 1, lidar_en: 1` → Full LIVO

---

## Launch File Architecture

Each launch file wires three nodes:

```
parameter_blackboard  ←  config/camera_*.yaml   (vikit intrinsics)
fastlivo_mapping      ←  config/*.yaml           (all SLAM parameters)
image_transport republish                        (compressed → raw)
```

`parameter_blackboard` (from `demo_nodes_cpp`) is the global parameter server workaround — vikit has no native ROS2 param support.

**Template**: `launch/mapping_ouster_scout.launch.py`

**Available launch files**:
| File | Sensor combo |
|------|-------------|
| `mapping_ouster_scout.launch.py` | Ouster OS1-64 + RealSense (Scout robot) |
| `mapping_aviz.launch.py` | Livox Avia + pinhole camera |
| `mapping_avia_marslvig.launch.py` | Livox Avia + MARS LVIG dataset |
| `mapping_ouster_ntu.launch.py` | Ouster + NTU VIRAL dataset |
| `mapping_scout_noeun.launch.py` | Scout robot, Noeun site |

---

## Configuration YAML Format

All main config files require the `/**:` namespace wrapper:

```yaml
/**:
  ros__parameters:
    common:
      img_topic: "/camera/camera/color/image_raw"
      lid_topic: "/ouster/points"
      imu_topic: "/ouster/imu"
      img_en: 1
      lidar_en: 1
      ros_driver_bug_fix: true   # true for ros-$DISTRO-ros2-ouster driver
```

Camera YAMLs also use `/**:` wrapper but are flat key-value (read by vikit):

```yaml
/**:
  ros__parameters:
    cam_model: Pinhole
    cam_width: 1280
    cam_height: 720
    cam_fx: 644.3
    cam_fy: 643.6
    cam_cx: 644.4
    cam_cy: 369.7
    cam_d0: ...
```

> ROS1 configs in `origin/neural_mapping` do **not** use the `/**:` wrapper — do not copy them directly.

---

## Critical Config Parameters

| Parameter | Notes |
|-----------|-------|
| `preprocess.lidar_type` | Use enum values above |
| `preprocess.scan_line` | Must match actual channel count (16, 32, 64, 128…) |
| `common.ros_driver_bug_fix` | `true` for `ros-$DISTRO-ros2-ouster`; `false` otherwise |
| `extrin_calib.Rcl` | LiDAR-to-camera rotation (row-major 3×3) |
| `extrin_calib.Pcl` | LiDAR-to-camera translation (metres) |
| `time_offset.img_time_offset` | Camera timestamp offset relative to LiDAR (seconds) |

---

## Adding a New Sensor Config

1. Copy an existing main config (e.g., `config/scout_thor.yaml`); update topics, `lidar_type`, `scan_line`, extrinsics, time offsets.
2. Copy `config/camera_scout.yaml`; update intrinsics (`cam_fx/fy/cx/cy`, distortion).
3. Copy `launch/mapping_ouster_scout.launch.py`; point at the new YAMLs and rviz config.

Reference values: `origin/neural_mapping` branch has ROS1 configs for Scout+Ouster+RealSense at Noeun station.

---

## Running

```bash
# Scout + Ouster OS1-64 + RealSense
ros2 launch fast_livo mapping_ouster_scout.launch.py use_rviz:=True

# Bag playback with simulated clock
ros2 bag play /path/to/bag --clock

# Livox Avia demo
ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=True
```

---

## Output

- `Log/` — git-ignored; PCD maps, pose output, evaluation results written here.
- Evaluation: `Log/result/ntu_viral/evaluate_viral.py`
- PCD save: controlled by `pcd_save.pcd_save_en` and `pcd_save.interval` in config.
- COLMAP output: `pcd_save.colmap_output_en: true` (requires `interval: -1`).

---

## Notes

- ROS1→ROS2 bag conversion: `pip install rosbags && rosbags-convert --src foo.bag --dst foo/`
- For Livox bags from ROS1: change `livox_ros_driver/msg/CustomMsg` → `livox_ros_driver2/msg/CustomMsg` in `metadata.yaml`.
- `ROOT_DIR` macro in CMake points to this package directory; used for `Log/` paths.
- Commercial use requires written permission from HKU MARS Lab (GPLv2 otherwise).
