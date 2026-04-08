#!/usr/bin/env python3
"""
Interactive LiDAR-camera extrinsic calibration tuner.
Keys are read from the OpenCV window.
Keep focus on the OpenCV window and press keys there.

Controls
────────
Translation (Pcl)               Rotation (Rcl delta)
  h / l   camera X  left/right    a / d   yaw   (around cam Y)
  k / j   camera Y  up/down       q / e   pitch (around cam X)
  u / o   camera Z  near/far      r / f   roll  (around cam Z)

Step size
  + / =   increase step            -      decrease step

Navigation
  n / p   next / prev frame

Output
  s       save current Rcl & Pcl to calib_tuned.txt
  ESC / ctrl-c   quit
"""

import sys, os, threading
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

import numpy as np
import cv2
import rosbag
import sensor_msgs.point_cloud2 as pc2

# ── initial calibration (calib_scout.txt) ────────────────────────────────────
K = np.array([[644.8412476, 0,           644.40625  ],
              [0,           644.1698608, 369.7319031],
              [0,           0,           1          ]], dtype=np.float64)

dist = np.array([-0.05756863206624985,  0.06629723310470581,
                  0.0006150348926894367, 0.0007578100194223225], dtype=np.float64)

Rcl = np.array([[ 0.019881, -0.999792, -0.004576],
                [-0.160109,  0.001334, -0.987098],
                [ 0.986899,  0.020357, -0.160049]], dtype=np.float64)

Pcl = np.array([-0.000026, -0.021708, -0.098460], dtype=np.float64)

BAG            = '/ws/data/nouen/scout/ros1_converted/my_camera_bag_20260317_070913.bag'
OUT_CALIB      = '/ws/data/nouen/scout/calib_tuned.txt'
EXTRACTED_DIR  = '/ws/data/nouen/scout/calib_frames'   # set by extract_calib_frames.py
DOT_RADIUS     = 2    # projected point dot size in pixels
DOT_ALPHA      = 0.55  # 0=fully transparent, 1=fully opaque

# ── load frames (from pre-extracted dir or directly from bag) ─────────────────
if os.path.isfile(os.path.join(EXTRACTED_DIR, 'frames.txt')):
    # Fast path: load pre-extracted PNGs + numpy clouds
    print(f'Loading pre-extracted frames from {EXTRACTED_DIR} ...')
    with open(os.path.join(EXTRACTED_DIR, 'frames.txt')) as f:
        meta = [l for l in f if not l.startswith('#') and l.strip()]
    n_frames = len(meta)
    # Store as list of (img_bgr, pts_float32) loaded lazily via get_frame()
    _extracted = True
    print(f'  {n_frames} frames ready')
else:
    print(f'Pre-extracted frames not found at {EXTRACTED_DIR}.')
    print('Loading bag (slow) — run extract_calib_frames.py first for faster startup.')
    _extracted = False
    img_buf, lid_buf = [], []
    with rosbag.Bag(BAG) as bag:
        for topic, msg, t in bag.read_messages(
                topics=['/camera/camera/color/image_raw', '/ouster/points']):
            s = msg.header.stamp.to_nsec()
            (img_buf if 'image' in topic else lid_buf).append((s, msg))
    img_stamps = np.array([x[0] for x in img_buf])
    lid_buf = lid_buf[::5]
    n_frames = len(lid_buf)
    print(f'  {n_frames} lidar frames  |  {len(img_buf)} images')

# ── helpers ───────────────────────────────────────────────────────────────────

def rot_matrix(axis, deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    if axis == 'x': return np.array([[1,0,0],[0,c,-s],[0,s,c]])
    if axis == 'y': return np.array([[c,0,s],[0,1,0],[-s,0,c]])
    if axis == 'z': return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def project(cloud_msg, img_bgr, Rcl_, Pcl_):
    # cloud_msg may be a ROS PointCloud2 message or a pre-decoded Nx3 float32 array
    if isinstance(cloud_msg, np.ndarray):
        pts = cloud_msg
    else:
        pts = np.array(list(pc2.read_points(
            cloud_msg, field_names=('x','y','z'), skip_nans=True)), np.float32)
    if len(pts) == 0:
        return img_bgr.copy()
    pts_cam = (Rcl_ @ pts[:,:3].T).T + Pcl_
    mask = pts_cam[:,2] > 0.1
    pts_cam = pts_cam[mask]
    if len(pts_cam) == 0:
        return img_bgr.copy()
    uv, _ = cv2.projectPoints(pts_cam.astype(np.float64),
                               np.zeros(3), np.zeros(3), K, dist)
    uv = np.round(uv.reshape(-1, 2)).astype(int)
    h, w = img_bgr.shape[:2]

    # filter to valid pixel coords
    valid = (uv[:,0] >= 0) & (uv[:,0] < w) & (uv[:,1] >= 0) & (uv[:,1] < h)
    uv     = uv[valid]
    depths = pts_cam[valid, 2]

    d_min = depths.min()
    d_max = np.percentile(depths, 95)
    idx   = np.clip((depths - d_min) / (d_max - d_min + 1e-6), 0, 1)
    idx   = (idx * 255).astype(np.uint8)
    colors = cv2.applyColorMap(idx.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)

    out = img_bgr.copy()
    r = DOT_RADIUS
    for dy in range(-r, r+1):
        for dx in range(-r, r+1):
            if dx*dx + dy*dy > r*r:
                continue
            vs = np.clip(uv[:,1] + dy, 0, h-1)
            us = np.clip(uv[:,0] + dx, 0, w-1)
            orig = out[vs, us].astype(np.float32)
            out[vs, us] = (DOT_ALPHA * colors + (1.0 - DOT_ALPHA) * orig).astype(np.uint8)
    return out


def draw_hud(img, Pcl_, Rcl_, step_t, step_r, frame_idx, last_key):
    h, w = img.shape[:2]
    sy    = np.sqrt(Rcl_[0,0]**2 + Rcl_[1,0]**2)
    pitch = np.degrees(np.arctan2(-Rcl_[2,0], sy))
    yaw   = np.degrees(np.arctan2( Rcl_[1,0], Rcl_[0,0]))
    roll  = np.degrees(np.arctan2( Rcl_[2,1], Rcl_[2,2]))
    lines = [
        f'Frame {frame_idx}/{n_frames-1}   last key: {repr(last_key)}',
        f'Pcl  x:{Pcl_[0]:+.5f}  y:{Pcl_[1]:+.5f}  z:{Pcl_[2]:+.5f} m',
        f'Rot  pitch:{pitch:+.3f}  yaw:{yaw:+.3f}  roll:{roll:+.3f} deg',
        f'Step t={step_t*1000:.2f}mm  r={step_r:.4f}deg',
        'h/l=X  k/j=Y  u/o=Z  a/d=yaw  q/e=pitch  r/f=roll  +/-=step  n/p=frame  s=save',
    ]
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, 18*len(lines)+10), (0,0,0), -1)
    cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)
    for i, l in enumerate(lines):
        cv2.putText(img, l, (6, 18*(i+1)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0,255,0), 1, cv2.LINE_AA)
    return img


def get_frame(idx):
    if _extracted:
        img_bgr = cv2.imread(os.path.join(EXTRACTED_DIR, 'images', f'{idx:05d}.png'))
        pts     = np.load(os.path.join(EXTRACTED_DIR, 'clouds', f'{idx:05d}.npy'))
        return img_bgr, pts   # pts is Nx3 float32 array (not a ROS msg)
    else:
        ls, lmsg = lid_buf[idx]
        ii = int(np.argmin(np.abs(img_stamps - ls)))
        _, imsg = img_buf[ii]
        img = np.frombuffer(imsg.data, np.uint8).reshape(imsg.height, imsg.width, 3).copy()
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR), lmsg


def save_calib(Rcl_, Pcl_):
    r, p = Rcl_.flatten(), Pcl_
    with open(OUT_CALIB, 'w') as f:
        f.write('# Tuned with calib_tune.py\n')
        f.write(f'Tr_velo_to_cam: '
                f'{r[0]:.6f} {r[1]:.6f} {r[2]:.6f} {p[0]:.6f} '
                f'{r[3]:.6f} {r[4]:.6f} {r[5]:.6f} {p[1]:.6f} '
                f'{r[6]:.6f} {r[7]:.6f} {r[8]:.6f} {p[2]:.6f}\n\n')
        f.write('# For scout_noeun_station.yaml:\n')
        f.write(f'Rcl: [{r[0]:.6f}, {r[1]:.6f}, {r[2]:.6f},\n')
        f.write(f'      {r[3]:.6f}, {r[4]:.6f}, {r[5]:.6f},\n')
        f.write(f'      {r[6]:.6f}, {r[7]:.6f}, {r[8]:.6f}]\n')
        f.write(f'Pcl: [{p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f}]\n')
    print(f'\nSaved → {OUT_CALIB}')


def get_key():
    """Read one keypress from OpenCV window."""
    k = cv2.waitKey(0)
    if k == -1:
        return ''
    
    # Handle escape sequence or special keys if necessary
    # For basic ascii, chr is enough
    c = k & 0xFF
    if c == 27:
        return '\x1b'
    try:
        return chr(c)
    except ValueError:
        return ''


# ── main loop ─────────────────────────────────────────────────────────────────

frame_idx = 0
step_t    = 0.005   # 5 mm
step_r    = 0.1     # degrees
last_key  = 'none'

cv2.namedWindow('LiDAR-Camera Calib', cv2.WINDOW_NORMAL)
cv2.resizeWindow('LiDAR-Camera Calib', 1280, 720)

raw_img, cloud_msg = get_frame(frame_idx)

print('\n=== Calib tuner ready — FOCUS THE OPENCV WINDOW AND TYPE ===')
print('h/l=X  k/j=Y  u/o=Z  a/d=yaw  q/e=pitch  r/f=roll')
print('+/-=step  n/p=frame  s=save  ESC=quit\n')

def render():
    vis = project(cloud_msg, raw_img, Rcl, Pcl)
    draw_hud(vis, Pcl, Rcl, step_t, step_r, frame_idx, last_key)
    cv2.imshow('LiDAR-Camera Calib', vis)

render()

try:
    while True:
        key = get_key()
        last_key = repr(key)

        # ESC or ctrl-c
        if key in ('\x1b', '\x03'):
            break

        changed = True

        # translation
        if   key == 'h':  Pcl[0] -= step_t
        elif key == 'l':  Pcl[0] += step_t
        elif key == 'k':  Pcl[1] -= step_t
        elif key == 'j':  Pcl[1] += step_t
        elif key == 'u':  Pcl[2] -= step_t
        elif key == 'o':  Pcl[2] += step_t

        # rotation
        elif key == 'a':  Rcl[:] = rot_matrix('y', -step_r) @ Rcl
        elif key == 'd':  Rcl[:] = rot_matrix('y',  step_r) @ Rcl
        elif key == 'q':  Rcl[:] = rot_matrix('x', -step_r) @ Rcl
        elif key == 'e':  Rcl[:] = rot_matrix('x',  step_r) @ Rcl
        elif key == 'r':  Rcl[:] = rot_matrix('z', -step_r) @ Rcl
        elif key == 'f':  Rcl[:] = rot_matrix('z',  step_r) @ Rcl

        # step size
        elif key in ('+', '='):
            step_t = min(step_t * 2, 0.5);    step_r = min(step_r * 2, 5.0)
        elif key == '-':
            step_t = max(step_t / 2, 0.0001); step_r = max(step_r / 2, 0.001)

        # navigation
        elif key == 'n':
            frame_idx = min(frame_idx + 1, n_frames - 1)
            raw_img, cloud_msg = get_frame(frame_idx)
        elif key == 'p':
            frame_idx = max(frame_idx - 1, 0)
            raw_img, cloud_msg = get_frame(frame_idx)

        # save
        elif key == 's':
            save_calib(Rcl, Pcl); changed = False

        else:
            changed = False

        if changed:
            sys.stdout.write(
                f'\rPcl=[{Pcl[0]:+.5f}, {Pcl[1]:+.5f}, {Pcl[2]:+.5f}]  '
                f'step={step_t*1000:.2f}mm/{step_r:.3f}deg  key={repr(key)}   ')
            sys.stdout.flush()
            render()

except KeyboardInterrupt:
    pass

cv2.destroyAllWindows()
print('\n\nFinal calibration:')
save_calib(Rcl, Pcl)
