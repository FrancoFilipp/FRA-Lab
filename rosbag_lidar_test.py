import rosbag
import numpy as np

#bag_l = rosbag.Bag("lidar/lidar_cono_10cm_2024-04-19-12-08-52.bag")
#bag_l = rosbag.Bag("lidar/lidar_cono_35cm_2024-04-19-12-10-38.bag")
bag_l = rosbag.Bag("lidar/lidar_cono_120cm_2024-04-19-12-15-58.bag")

values_range_list = []
for topic, msg, t in bag_l.read_messages():
    values_range_list.append(msg.ranges[178:182])
print(max(values_range_list))
print(min(values_range_list))
print(np.mean(values_range_list))
bag_l.close()