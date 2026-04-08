#!/usr/bin/env python3
"""
Project Ouster LiDAR points onto RGB image using calibration from
camera_scout_r64.yaml and scout_nouen_station.yaml.

Saves one output image per pair to /ws/data/nouen/scout/projection_output/
"""

import sys, os
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

import numpy as np
import cv2
import rosbag
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# ── calibration (from camera_scout_r64.yaml + scout_nouen_station.yaml) ───────

# Camera intrinsics  (P2 values)
fx, fy = 643.615356, 644.94525
cx, cy = 644.40625,  369.7319031
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]], dtype=np.float64)

# Distortion  (from RealSense camera_info, plumb_bob: k1 k2 p1 p2)
dist = np.array([-0.05756863206624985,
                  0.06629723310470581,
                  0.0006150348926894367,
                  0.0007578100194223225], dtype=np.float64)

# Extrinsics: LiDAR → camera  (Tr_velo_to_cam)
Rcl = np.array([[ 0.002736, -0.999996,  0.000435],
                [-0.254728, -0.001118, -0.967012],
                [ 0.967009,  0.002535, -0.254730]], dtype=np.float64)
Pcl = np.array([0.001859, -0.022877, -0.073405], dtype=np.float64)  # metres

# ── helpers ───────────────────────────────────────────────────────────────────

def ros_image_to_cv2(msg):
    """Convert sensor_msgs/Image (rgb8) to BGR numpy array."""
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img.copy()


def project_lidar(cloud_msg, img):
    """
    Project PointCloud2 onto img.
    Returns annotated image.
    """
    # Extract xyz from PointCloud2
    pts = np.array(list(pc2.read_points(cloud_msg, field_names=('x','y','z'),
                                         skip_nans=True)), dtype=np.float32)
    if pts.shape[0] == 0:
        return img

    xyz = pts[:, :3]  # (N, 3)

    # Transform to camera frame:  p_cam = Rcl @ p_lidar + Pcl
    pts_cam = (Rcl @ xyz.T).T + Pcl   # (N, 3)

    # Keep only points in front of camera (z > 0.1 m)
    mask = pts_cam[:, 2] > 0.1
    pts_cam = pts_cam[mask]
    if pts_cam.shape[0] == 0:
        return img

    # Project with distortion using OpenCV
    rvec = np.zeros(3, dtype=np.float64)
    tvec = np.zeros(3, dtype=np.float64)
    pts_2d, _ = cv2.projectPoints(pts_cam.astype(np.float64),
                                   rvec, tvec, K, dist)
    pts_2d = pts_2d.reshape(-1, 2)

    h, w = img.shape[:2]
    depths = pts_cam[:, 2]
    d_min, d_max = depths.min(), np.percentile(depths, 95)

    out = img.copy()
    for (u, v), d in zip(pts_2d, depths):
        u, v = int(round(u)), int(round(v))
        if not (0 <= u < w and 0 <= v < h):
            continue
        # Colour by depth: near=red, far=blue (jet colormap)
        t = float(np.clip((d - d_min) / (d_max - d_min + 1e-6), 0, 1))
        color = cv2.applyColorMap(
            np.array([[int(t * 255)]], dtype=np.uint8), cv2.COLORMAP_JET)[0, 0].tolist()
        cv2.circle(out, (u, v), 2, color, -1)

    # Depth bar legend
    bar_h = h - 40
    for i in range(200):
        t = i / 199.0
        c = cv2.applyColorMap(np.array([[int(t*255)]], dtype=np.uint8),
                               cv2.COLORMAP_JET)[0,0].tolist()
        cv2.rectangle(out, (w-230+i, bar_h), (w-230+i+1, bar_h+15), c, -1)
    cv2.putText(out, f'{d_min:.1f}m', (w-235, bar_h+30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.putText(out, f'{d_max:.1f}m', (w-40, bar_h+30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.putText(out, 'depth', (w-140, bar_h-5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1)

    n_proj = int(np.sum((pts_2d[:,0] >= 0) & (pts_2d[:,0] < w) &
                        (pts_2d[:,1] >= 0) & (pts_2d[:,1] < h)))
    cv2.putText(out, f'pts projected: {n_proj}', (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    return out


# ── main ──────────────────────────────────────────────────────────────────────

BAG  = '/ws/data/nouen/scout/ros1_converted/my_camera_bag_20260317_070913.bag'
OUT  = '/ws/data/nouen/scout/projection_output'
N_SAVE = 10   # number of frames to save

os.makedirs(OUT, exist_ok=True)

IMG_TOPIC   = '/camera/camera/color/image_raw'
LIDAR_TOPIC = '/ouster/points'

print(f'Reading: {BAG}')

# Buffer: collect (stamp_ns, msg) pairs for each topic, then match nearest
img_buf   = []   # list of (stamp_ns, msg)
lidar_buf = []   # list of (stamp_ns, msg)

print('Buffering messages ...')
with rosbag.Bag(BAG) as bag:
    for topic, msg, t in bag.read_messages(topics=[IMG_TOPIC, LIDAR_TOPIC]):
        stamp_ns = msg.header.stamp.to_nsec()
        if topic == IMG_TOPIC:
            img_buf.append((stamp_ns, msg))
        else:
            lidar_buf.append((stamp_ns, msg))

print(f'  images : {len(img_buf)}')
print(f'  lidar  : {len(lidar_buf)}')

# Sort by stamp
img_buf.sort(key=lambda x: x[0])
lidar_buf.sort(key=lambda x: x[0])

img_stamps = np.array([x[0] for x in img_buf])

saved = 0
step  = max(1, len(lidar_buf) // N_SAVE)

for i in range(0, len(lidar_buf), step):
    if saved >= N_SAVE:
        break
    lidar_stamp, lidar_msg = lidar_buf[i]

    # Nearest image by header stamp
    idx = int(np.argmin(np.abs(img_stamps - lidar_stamp)))
    dt_ms = abs(img_stamps[idx] - lidar_stamp) / 1e6
    _, img_msg = img_buf[idx]

    img_cv  = ros_image_to_cv2(img_msg)
    out_img = project_lidar(lidar_msg, img_cv)

    fname = os.path.join(OUT, f'frame_{saved:03d}_dt{dt_ms:.1f}ms.jpg')
    cv2.imwrite(fname, out_img, [cv2.IMWRITE_JPEG_QUALITY, 95])
    print(f'  saved {fname}  (dt={dt_ms:.1f} ms)')
    saved += 1

print(f'\nDone. {saved} images saved to {OUT}/')
