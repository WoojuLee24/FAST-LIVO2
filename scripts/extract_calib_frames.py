#!/usr/bin/env python3
"""
Extract synced RGB + LiDAR frames from a ROS1 bag for calib_tune.py.

Writes to OUT_DIR:
  images/  *.png          (BGR, decoded from image_raw)
  clouds/  *.npy          (float32 Nx3 xyz)
  frames.txt              (index  lidar_stamp_ns  image_stamp_ns)

Usage:
  python3 extract_calib_frames.py
"""

import sys, os
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

import numpy as np
import cv2
import rosbag
import sensor_msgs.point_cloud2 as pc2

BAG       = '/ws/data/nouen/scout/ros1_converted/my_camera_bag_20260317_070913.bag'
OUT_DIR   = '/ws/data/nouen/scout/calib_frames'
IMG_TOPIC = '/camera/camera/color/image_raw'
LID_TOPIC = '/ouster/points'
LIDAR_STRIDE = 5   # keep every Nth lidar frame (same as calib_tune.py)

os.makedirs(f'{OUT_DIR}/images', exist_ok=True)
os.makedirs(f'{OUT_DIR}/clouds', exist_ok=True)

print(f'Reading bag: {BAG}')
print('Buffering image timestamps ...')

img_buf = []   # [(stamp_ns, msg)]
lid_buf = []   # [(stamp_ns, msg)]

with rosbag.Bag(BAG) as bag:
    total = bag.get_message_count(topic_filters=[IMG_TOPIC, LID_TOPIC])
    done  = 0
    for topic, msg, t in bag.read_messages(topics=[IMG_TOPIC, LID_TOPIC]):
        s = msg.header.stamp.to_nsec()
        if IMG_TOPIC in topic:
            img_buf.append((s, msg))
        else:
            lid_buf.append((s, msg))
        done += 1
        if done % 500 == 0:
            print(f'  read {done}/{total} messages  '
                  f'({len(img_buf)} imgs, {len(lid_buf)} lidar)', end='\r')

print(f'\nTotal: {len(img_buf)} images, {len(lid_buf)} lidar frames')

lid_buf = lid_buf[::LIDAR_STRIDE]
print(f'After stride-{LIDAR_STRIDE}: {len(lid_buf)} lidar frames to extract')

img_stamps = np.array([x[0] for x in img_buf])

meta_lines = []

for i, (ls, lmsg) in enumerate(lid_buf):
    # find closest image
    ii = int(np.argmin(np.abs(img_stamps - ls)))
    is_, imsg = img_buf[ii]

    # decode image
    img = np.frombuffer(imsg.data, np.uint8).reshape(imsg.height, imsg.width, 3)
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # decode point cloud
    pts = np.array(list(pc2.read_points(
        lmsg, field_names=('x', 'y', 'z'), skip_nans=True)), np.float32)

    # save
    cv2.imwrite(f'{OUT_DIR}/images/{i:05d}.png', img_bgr)
    np.save(f'{OUT_DIR}/clouds/{i:05d}.npy', pts)
    meta_lines.append(f'{i} {ls} {is_}')

    if (i + 1) % 10 == 0 or i == len(lid_buf) - 1:
        print(f'  saved {i+1}/{len(lid_buf)} frames', end='\r')

with open(f'{OUT_DIR}/frames.txt', 'w') as f:
    f.write('# idx  lidar_stamp_ns  image_stamp_ns\n')
    f.write('\n'.join(meta_lines) + '\n')

print(f'\nDone. Extracted {len(lid_buf)} frames → {OUT_DIR}')
