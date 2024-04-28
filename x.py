import rosbag
import numpy as np

bag_l = rosbag.Bag("lidar/lidar_cono_120cm_2024-04-19-12-15-58.bag")

# For all scans, print if there is a scan with all zeros
for topic, scan, t in bag_l.read_messages(topics=['/scan']):
    if np.all(scan.ranges == 0.0):
        print("All zeros")
