#!/usr/bin/env python3
"""
Convert ROS2 MCAP bag to ROS1 bag.
Handles standard sensor_msgs, tf2_msgs, nav_msgs, std_msgs.
Skips ROS2-only types: realsense2_camera_msgs, negotiated_interfaces, rcl_interfaces.
"""

import sys
import os

# Source ROS1 Noetic environment
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

import rosbag
from mcap_ros2.reader import read_ros2_messages
from mcap.reader import make_reader

# ROS1 message types
from std_msgs.msg import String, Header
from sensor_msgs.msg import Imu, Image, CameraInfo, PointCloud2, CompressedImage, PointField
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import (TransformStamped, Transform, Vector3, Quaternion,
                                 TwistWithCovariance, Twist, PoseWithCovariance,
                                 Pose, Point)
from nav_msgs.msg import Odometry
import rospy

# Topics with no ROS1 equivalent — skip them
SKIP_TYPES = {
    'realsense2_camera_msgs/msg/Metadata',
    'negotiated_interfaces/msg/NegotiatedTopicsInfo',
    'rcl_interfaces/msg/ParameterEvent',
}


def to_ros1_time(stamp):
    """Convert ROS2 Time object to rospy.Time."""
    return rospy.Time(stamp.sec, stamp.nanosec)


def convert_header(h2):
    h1 = Header()
    h1.stamp = to_ros1_time(h2.stamp)
    h1.frame_id = h2.frame_id
    return h1


def convert_vector3(v):
    return Vector3(x=v.x, y=v.y, z=v.z)


def convert_quaternion(q):
    return Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)


def convert_imu(m):
    msg = Imu()
    msg.header = convert_header(m.header)
    msg.orientation = convert_quaternion(m.orientation)
    msg.orientation_covariance = list(m.orientation_covariance)
    msg.angular_velocity = convert_vector3(m.angular_velocity)
    msg.angular_velocity_covariance = list(m.angular_velocity_covariance)
    msg.linear_acceleration = convert_vector3(m.linear_acceleration)
    msg.linear_acceleration_covariance = list(m.linear_acceleration_covariance)
    return msg


def convert_image(m):
    msg = Image()
    msg.header = convert_header(m.header)
    msg.height = m.height
    msg.width = m.width
    msg.encoding = m.encoding
    msg.is_bigendian = m.is_bigendian
    msg.step = m.step
    msg.data = bytes(m.data)
    return msg


def convert_camera_info(m):
    msg = CameraInfo()
    msg.header = convert_header(m.header)
    msg.height = m.height
    msg.width = m.width
    msg.distortion_model = m.distortion_model
    msg.D = list(m.d)
    msg.K = list(m.k)
    msg.R = list(m.r)
    msg.P = list(m.p)
    msg.binning_x = m.binning_x
    msg.binning_y = m.binning_y
    msg.roi.x_offset = m.roi.x_offset
    msg.roi.y_offset = m.roi.y_offset
    msg.roi.height = m.roi.height
    msg.roi.width = m.roi.width
    msg.roi.do_rectify = m.roi.do_rectify
    return msg


def convert_pointfield(f):
    pf = PointField()
    pf.name = f.name
    pf.offset = f.offset
    pf.datatype = f.datatype
    pf.count = f.count
    return pf


def convert_pointcloud2(m):
    msg = PointCloud2()
    msg.header = convert_header(m.header)
    msg.height = m.height
    msg.width = m.width
    msg.fields = [convert_pointfield(f) for f in m.fields]
    msg.is_bigendian = m.is_bigendian
    msg.point_step = m.point_step
    msg.row_step = m.row_step
    msg.data = bytes(m.data)
    msg.is_dense = m.is_dense
    return msg


def convert_compressed_image(m):
    msg = CompressedImage()
    msg.header = convert_header(m.header)
    msg.format = m.format
    msg.data = bytes(m.data)
    return msg


def convert_transform_stamped(ts):
    t = TransformStamped()
    t.header = convert_header(ts.header)
    t.child_frame_id = ts.child_frame_id
    t.transform.translation = convert_vector3(ts.transform.translation)
    t.transform.rotation = convert_quaternion(ts.transform.rotation)
    return t


def convert_tf_message(m):
    msg = TFMessage()
    msg.transforms = [convert_transform_stamped(ts) for ts in m.transforms]
    return msg


def convert_odometry(m):
    msg = Odometry()
    msg.header = convert_header(m.header)
    msg.child_frame_id = m.child_frame_id
    # pose
    msg.pose.pose.position = Point(
        x=m.pose.pose.position.x,
        y=m.pose.pose.position.y,
        z=m.pose.pose.position.z,
    )
    msg.pose.pose.orientation = convert_quaternion(m.pose.pose.orientation)
    msg.pose.covariance = list(m.pose.covariance)
    # twist
    msg.twist.twist.linear = convert_vector3(m.twist.twist.linear)
    msg.twist.twist.angular = convert_vector3(m.twist.twist.angular)
    msg.twist.covariance = list(m.twist.covariance)
    return msg


def convert_string(m):
    msg = String()
    msg.data = m.data
    return msg


TYPE_CONVERTERS = {
    'sensor_msgs/msg/Imu': convert_imu,
    'sensor_msgs/msg/Image': convert_image,
    'sensor_msgs/msg/CameraInfo': convert_camera_info,
    'sensor_msgs/msg/PointCloud2': convert_pointcloud2,
    'sensor_msgs/msg/CompressedImage': convert_compressed_image,
    'tf2_msgs/msg/TFMessage': convert_tf_message,
    'nav_msgs/msg/Odometry': convert_odometry,
    'std_msgs/msg/String': convert_string,
}


def main():
    src = '/ws/data/nouen/scout/my_camera_bag_20260317_070913'
    mcap_file = os.path.join(src, 'my_camera_bag_20260317_070913_0.mcap')
    dst = '/ws/data/nouen/scout/ros1_converted/my_camera_bag_20260317_070913.bag'

    os.makedirs(os.path.dirname(dst), exist_ok=True)

    # Count total messages for progress
    total = 46045
    count = 0
    skipped_types = set()

    print(f"Converting: {mcap_file}")
    print(f"Output:     {dst}")

    with rosbag.Bag(dst, 'w') as bag:
        with open(mcap_file, 'rb') as f:
            for msg_obj in read_ros2_messages(f):
                schema_name = msg_obj.schema.name if msg_obj.schema else None
                topic = msg_obj.channel.topic
                ros_msg = msg_obj.ros_msg
                log_time = rospy.Time(nsecs=msg_obj.log_time_ns)

                if schema_name in SKIP_TYPES:
                    skipped_types.add(schema_name)
                    count += 1
                    continue

                converter = TYPE_CONVERTERS.get(schema_name)
                if converter is None:
                    if schema_name not in skipped_types:
                        print(f"  WARNING: No converter for {schema_name} on {topic}, skipping")
                        skipped_types.add(schema_name)
                    count += 1
                    continue

                try:
                    ros1_msg = converter(ros_msg)
                    bag.write(topic, ros1_msg, log_time)
                except Exception as e:
                    print(f"  ERROR converting {topic} ({schema_name}): {e}")
                    count += 1
                    continue

                count += 1
                if count % 2000 == 0:
                    print(f"  Progress: {count}/{total} ({100*count//total}%)")

    print(f"\nDone! Converted {count} messages.")
    if skipped_types:
        print(f"Skipped types: {skipped_types}")
    print(f"Output: {dst}")


if __name__ == '__main__':
    main()
