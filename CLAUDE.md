# FAST-LIVO2 — CLAUDE.md

## Overview

FAST-LIVO2 is a **Fast, Direct LiDAR-Inertial-Visual Odometry** system performing tightly-coupled multi-sensor fusion for real-time 3D localization and mapping. Published in T-RO 2024. ROS package name: `fast_livo`.

**Primary executable**: `fastlivo_mapping` (node name: `laserMapping`)

---

## Workflow: FAST-LIVO2 → GS-SDF

FAST-LIVO2 processes raw sensor bags and outputs a `fast_livo2_bag`, which GS-SDF then uses for neural mapping / video rendering.

### Step 1 — Raw bag → fast_livo2_bag (processing)

```bash
# Terminal 1: launch FAST-LIVO2
roslaunch fast_livo mapping_avia.launch

# Terminal 2: record output topics
rosbag record /aft_mapped_to_init /cloud_registered /rgb_img /tf /tf_static /path \
  -b 4096 -O fast_livo2_campus.bag

# Terminal 3: play raw input bag
rosbag play campus.bag
```

### Step 2 — fast_livo2_bag → video (GS-SDF)

```bash
rosrun neural_mapping neural_mapping_node train \
  src/GS-SDF/config/fast_livo/campus.yaml \
  /ws/data/FAST_LIVO2_Datasets/bags_fastlivo2/fast_livo2_campus.bag
```

---

## Build

```bash
# From workspace root
catkin build fast_livo

# Or inside the package directory
catkin build --this
```

**Requirements**: C++17, Eigen3 (≥3.3.4), PCL (≥1.8), OpenCV (≥4.2), Sophus, Boost, vikit_common, vikit_ros (rpg_vikit fork)

**Compiler flags** are architecture-aware (set automatically in CMakeLists.txt):
- x86-64: `-O3 -march=native -mtune=native -funroll-loops`
- ARM64: `-O3 -mcpu=native -mtune=native -ffast-math`
- ARM32+NEON: `-O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math`

**Optional**: OpenMP (parallelization), mimalloc (faster allocation)

---

## Running

```bash
# Primary launch (Livox AVIA + pinhole camera)
roslaunch fast_livo mapping_avia.launch

# Other sensor configs
roslaunch fast_livo mapping_avia_marslvig.launch
roslaunch fast_livo mapping_ouster_ntu.launch
roslaunch fast_livo mapping_hesaixt32_hilti22.launch
```

---

## Architecture

### Module Map

| File | Role |
|------|------|
| `src/main.cpp` | ROS node init, creates LIVMapper, starts run loop |
| `src/LIVMapper.cpp` + `include/LIVMapper.h` | Central coordinator: sensor sync, EKF orchestration, ROS I/O |
| `src/vio.cpp` + `include/vio.h` | Visual-Inertial Odometry: direct patch tracking, camera EKF updates |
| `src/voxel_map.cpp` + `include/voxel_map.h` | Octree voxel map: plane fitting, point-to-plane residuals |
| `src/IMU_Processing.cpp` + `include/IMU_Processing.h` | IMU preintegration, bias estimation, point cloud undistortion |
| `src/preprocess.cpp` + `include/preprocess.h` | LiDAR preprocessing: multi-sensor support, feature extraction, downsampling |
| `src/frame.cpp` + `include/frame.h` | Camera frame with image pyramid |
| `src/visual_point.cpp` + `include/visual_point.h` | 3D visual map points with normals and covariances |
| `include/common_lib.h` | Core types: `StatesGroup` (19-DoF state), `MeasureGroup`, `LidarMeasureGroup` |
| `include/feature.h` | Visual feature: 2D pixel, 3D bearing vector, patch, link to VisualPoint |

### CMake Libraries

- `vio` — vio.cpp, frame.cpp, visual_point.cpp
- `lio` — voxel_map.cpp
- `pre` — preprocess.cpp
- `imu_proc` — IMU_Processing.cpp
- `laser_mapping` — LIVMapper.cpp (links all above)

### State Vector (`StatesGroup`, 19 DoF)

Rotation (SO3) + position + velocity + IMU accel bias + IMU gyro bias + gravity vector + camera exposure time

### Processing Pipeline

1. **Sync** — Buffer and align LiDAR/IMU/image data (`MeasureGroup`)
2. **Preprocess** — LiDAR classification and downsampling (`Preprocess`)
3. **IMU undistortion** — Preintegrate IMU to undistort point cloud (`ImuProcess`)
4. **LIO update** — Build point-to-plane residuals in voxel map, EKF update
5. **VIO update** — Direct patch tracking on image pyramid, EKF update
6. **Map update** — Insert new points into octree voxel map
7. **Publish** — Odometry, path, clouds, planes to RViZ

---

## Configuration

All parameters are YAML, loaded at launch — no recompilation needed.

### Sensor Configs (`config/`)

| File | Sensor Suite |
|------|-------------|
| `avia.yaml` | Livox AVIA + pinhole camera (primary) |
| `MARS_LVIG.yaml` | Mars rover sensor suite |
| `NTU_VIRAL.yaml` | NTU VIRAL benchmark dataset |
| `HILTI22.yaml` | Hilti 2022 dataset |

### Camera Intrinsic Configs (`config/`)

| File | Model |
|------|-------|
| `camera_pinhole.yaml` | Standard pinhole |
| `camera_MARS_LVIG.yaml` | Mars rover |
| `camera_NTU_VIRAL.yaml` | NTU dataset |
| `camera_fisheye_HILTI22.yaml` | Fisheye lens |

### Key Parameters (avia.yaml)

```yaml
# Topics
img_topic: /left_camera/image
lid_topic: /livox/lidar
imu_topic: /livox/imu

# Mode (1=ONLY_LO, 2=ONLY_LIO, 3=LIVO)
img_en: 1
lidar_en: 1

# Extrinsics (LiDAR-to-IMU)
extrinsic_T: [0.04165, 0.02326, -0.0284]
extrinsic_R: [1,0,0, 0,1,0, 0,0,1]

# Time offsets
imu_time_offset: 0.0
img_time_offset: 0.1

# LiDAR preprocessing
lidar_type: 1        # 1=Livox AVIA, 2=VLP-16, 3=Ouster64, etc.
filter_size_surf: 0.1
blind: 0.8

# VIO
patch_size: 8
patch_pyrimid_level: 4
max_iterations: 5    # VIO EKF iterations
normal_en: true
exposure_estimate_en: true

# LIO
voxel_size: 0.5
max_layer: 2
max_points_num: 50

# Output
pcd_save_en: false
pose_output_en: false
```

### Supported LiDAR Types (`lidar_type`)

| Value | Sensor |
|-------|--------|
| 1 | Livox AVIA |
| 2 | Velodyne VLP-16 |
| 3 | Ouster 64 |
| 4 | Intel L515 |
| 5 | Hesai XT32 |
| 6 | Hesai Pandar128 |
| 7 | Robosense Airy |

---

## Key Classes

### `LIVMapper` (`LIVMapper.h/cpp`)
Central node class. Owns all sub-managers. Key method: `stateEstimationAndMapping()`.

### `VIOManager` (`vio.h/cpp`)
Direct visual tracking via affine-warped patches. Key methods:
- `processFrame()` — full VIO update for one image
- `computeJacobianAndUpdateEKF()` — visual EKF measurement update
- `getWarpMatrixAffine()` / `warpAffine()` — patch warping

### `VoxelMapManager` / `VoxelOctoTree` (`voxel_map.h/cpp`)
Hash-indexed octree (keyed by 3D voxel coordinates). Each node stores a `VoxelPlane` for point-to-plane residuals.

### `ImuProcess` (`IMU_Processing.h/cpp`)
IMU preintegration and bias estimation. Call `Process()` to undistort a scan.

### `Preprocess` (`preprocess.h/cpp`)
LiDAR-type-aware preprocessing. Call `process()` to convert raw messages to classified `PointCloudXYZI`.

---

## RViZ

Pre-configured layouts in `rviz_cfg/`:
- `fast_livo2.rviz` — default (use with avia launch)
- `M300.rviz` — DJI M300
- `hilti.rviz` — Hilti dataset
- `ntu_viral.rviz` — NTU VIRAL dataset

---

## Common Tasks

**Add support for a new LiDAR**: Extend `Preprocess::process()` in `src/preprocess.cpp` and add a new `lidar_type` value.

**Tune VIO tracking**: Adjust `patch_size`, `patch_pyrimid_level`, `max_iterations`, `outlier_threshold` in the sensor YAML.

**Tune LIO mapping**: Adjust `voxel_size`, `max_layer`, `max_points_num`, `min_eigen_value` in the sensor YAML.

**Enable local map sliding** (for long runs): Set `map_sliding_en: true`, `half_map_size`, and `sliding_thresh` in YAML.

**Save output**: Set `pcd_save_en: true` and/or `pose_output_en: true` in YAML. Output goes to `Log/`.

---

## License

GPLv2. Commercial use requires contact with developers (see README.md).
