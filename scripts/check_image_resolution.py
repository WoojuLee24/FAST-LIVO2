import rosbag
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()
bag = rosbag.Bag('/ws/data/FAST_LIVO2_Datasets/1bags_fastlivo2/campus_fastlivo2.bag')
for topic, msg, t in bag.read_messages(topics=['/origin_img']):
    img = bridge.imgmsg_to_cv2(msg)
    print(f"as-is  resolution: {img.shape}")  # (H, W, C)
    break
bag.close()

bag = rosbag.Bag('/ws/data/FAST_LIVO2_Datasets/bags_fastlivo2/fast_livo2_campus.bag')
for topic, msg, t in bag.read_messages(topics=['/origin_img']):
    img = bridge.imgmsg_to_cv2(msg)
    print(f"to-be  resolution: {img.shape}")
    break
bag.close()