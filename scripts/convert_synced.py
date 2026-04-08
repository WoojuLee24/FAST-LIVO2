#!/usr/bin/env python3
"""
Convert ROS2 MCAP bag to ROS1 bag with sync between /ouster/points and
/camera/camera/color/image_raw.

Topics written:
  /ouster/imu            — all messages (sensor_msgs/Imu)
  /ouster/points         — all 1240 scans (sensor_msgs/PointCloud2)
  /camera/camera/color/image_raw — one image per lidar scan, nearest in time
                                   (same count as /ouster/points)

Strategy:
  Pass 1: collect header timestamps for both lidar and camera (lightweight).
  Match:  for each lidar scan find the nearest camera frame; build the
          accepted set of camera header timestamps.
  Pass 2: stream all three topics and write only accepted messages.
"""

import sys, os, bisect

sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

import rosbag
import rospy
from mcap_ros2.reader import read_ros2_messages
from sensor_msgs.msg import Imu, Image, PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion

MCAP  = '/ws/data/nouen/scout/my_camera_bag_20260317_070913/my_camera_bag_20260317_070913_0.mcap'
DST   = '/ws/data/nouen/scout/ros1_converted/synced_ouster_camera.bag'

TOPICS_PASS1 = {'/ouster/points', '/camera/camera/color/image_raw'}
TOPICS_PASS2 = {'/ouster/points', '/ouster/imu', '/camera/camera/color/image_raw'}

MAX_SYNC_DIFF_NS = 100_000_000   # 100 ms hard limit; warn if exceeded


# ── converters ────────────────────────────────────────────────────────────────

def to_time(stamp):
    return rospy.Time(stamp.sec, stamp.nanosec)

def conv_header(h):
    r = Header(); r.stamp = to_time(h.stamp); r.frame_id = h.frame_id; return r

def conv_imu(m):
    r = Imu()
    r.header = conv_header(m.header)
    r.orientation = Quaternion(x=m.orientation.x, y=m.orientation.y,
                               z=m.orientation.z, w=m.orientation.w)
    r.orientation_covariance        = list(m.orientation_covariance)
    r.angular_velocity              = Vector3(x=m.angular_velocity.x,
                                              y=m.angular_velocity.y,
                                              z=m.angular_velocity.z)
    r.angular_velocity_covariance   = list(m.angular_velocity_covariance)
    r.linear_acceleration           = Vector3(x=m.linear_acceleration.x,
                                              y=m.linear_acceleration.y,
                                              z=m.linear_acceleration.z)
    r.linear_acceleration_covariance = list(m.linear_acceleration_covariance)
    return r

def conv_image(m):
    r = Image()
    r.header     = conv_header(m.header)
    r.height     = m.height
    r.width      = m.width
    r.encoding   = m.encoding
    r.is_bigendian = m.is_bigendian
    r.step       = m.step
    r.data       = bytes(m.data)
    return r

def conv_pointcloud2(m):
    r = PointCloud2()
    r.header = conv_header(m.header)
    r.height = m.height; r.width = m.width
    r.fields = [_pf(f) for f in m.fields]
    r.is_bigendian = m.is_bigendian
    r.point_step = m.point_step; r.row_step = m.row_step
    r.data = bytes(m.data); r.is_dense = m.is_dense
    return r

def _pf(f):
    p = PointField(); p.name = f.name; p.offset = f.offset
    p.datatype = f.datatype; p.count = f.count; return p


# ── helpers ────────────────────────────────────────────────────────────────────

def header_stamp_ns(ros_msg):
    """Return header.stamp as integer nanoseconds."""
    s = ros_msg.header.stamp
    return s.sec * 1_000_000_000 + s.nanosec


def nearest_index(sorted_list, value):
    """Index in sorted_list whose value is closest to `value`."""
    idx = bisect.bisect_left(sorted_list, value)
    if idx == 0:
        return 0
    if idx == len(sorted_list):
        return len(sorted_list) - 1
    before = sorted_list[idx - 1]
    after  = sorted_list[idx]
    return idx - 1 if (value - before) <= (after - value) else idx


# ── pass 1: collect timestamps ────────────────────────────────────────────────

def collect_timestamps():
    lidar_ts  = []   # nanoseconds, header stamp
    camera_ts = []   # nanoseconds, header stamp

    with open(MCAP, 'rb') as f:
        for msg in read_ros2_messages(f, topics=list(TOPICS_PASS1)):
            topic = msg.channel.topic
            ts    = header_stamp_ns(msg.ros_msg)
            if topic == '/ouster/points':
                lidar_ts.append(ts)
            elif topic == '/camera/camera/color/image_raw':
                camera_ts.append(ts)

    lidar_ts.sort()
    camera_ts.sort()
    return lidar_ts, camera_ts


# ── matching ───────────────────────────────────────────────────────────────────

def build_accepted_camera_set(lidar_ts, camera_ts):
    """
    For each lidar timestamp find the nearest camera timestamp.
    Returns a set of accepted camera header stamp values (ns).
    Also prints sync statistics.
    """
    accepted = set()
    diffs    = []

    for lt in lidar_ts:
        idx = nearest_index(camera_ts, lt)
        ct  = camera_ts[idx]
        diff = abs(lt - ct)
        accepted.add(ct)
        diffs.append(diff)

    diffs_ms = [d / 1e6 for d in diffs]
    print(f"  Sync stats over {len(lidar_ts)} pairs:")
    print(f"    mean  dt = {sum(diffs_ms)/len(diffs_ms):.1f} ms")
    print(f"    max   dt = {max(diffs_ms):.1f} ms")
    print(f"    > 100ms  = {sum(1 for d in diffs if d > MAX_SYNC_DIFF_NS)}")
    print(f"  Unique camera frames selected: {len(accepted)}  "
          f"(duplicates due to >1 lidar scan per camera frame: "
          f"{len(lidar_ts) - len(accepted)})")
    return accepted


# ── pass 2: write bag ─────────────────────────────────────────────────────────

def write_bag(accepted_camera_ts):
    os.makedirs(os.path.dirname(DST), exist_ok=True)

    counts = {'/ouster/points': 0, '/ouster/imu': 0,
              '/camera/camera/color/image_raw': 0}

    with rosbag.Bag(DST, 'w') as bag:
        with open(MCAP, 'rb') as f:
            for msg in read_ros2_messages(f, topics=list(TOPICS_PASS2)):
                topic    = msg.channel.topic
                log_time = rospy.Time(nsecs=msg.log_time_ns)

                if topic == '/ouster/imu':
                    bag.write(topic, conv_imu(msg.ros_msg), log_time)
                    counts[topic] += 1

                elif topic == '/ouster/points':
                    bag.write(topic, conv_pointcloud2(msg.ros_msg), log_time)
                    counts[topic] += 1

                elif topic == '/camera/camera/color/image_raw':
                    ts = header_stamp_ns(msg.ros_msg)
                    if ts in accepted_camera_ts:
                        bag.write(topic, conv_image(msg.ros_msg), log_time)
                        counts[topic] += 1

                total = sum(counts.values())
                if total % 2000 == 0 and total > 0:
                    print(f"  written {total} msgs — "
                          f"lidar:{counts['/ouster/points']}  "
                          f"imu:{counts['/ouster/imu']}  "
                          f"cam:{counts['/camera/camera/color/image_raw']}")

    return counts


# ── main ───────────────────────────────────────────────────────────────────────

def main():
    print(f"Source: {MCAP}")
    print(f"Output: {DST}\n")

    print("Pass 1 — collecting timestamps ...")
    lidar_ts, camera_ts = collect_timestamps()
    print(f"  /ouster/points                 : {len(lidar_ts)} scans")
    print(f"  /camera/camera/color/image_raw : {len(camera_ts)} frames\n")

    print("Matching lidar scans to nearest camera frames ...")
    accepted_camera_ts = build_accepted_camera_set(lidar_ts, camera_ts)
    print()

    print("Pass 2 — writing ROS1 bag ...")
    counts = write_bag(accepted_camera_ts)

    print(f"\nDone.")
    print(f"  /ouster/points                 : {counts['/ouster/points']}")
    print(f"  /camera/camera/color/image_raw : {counts['/camera/camera/color/image_raw']}")
    print(f"  /ouster/imu                    : {counts['/ouster/imu']}")
    print(f"  Output: {DST}")


if __name__ == '__main__':
    main()
