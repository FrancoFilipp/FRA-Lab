import rosbag
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

DISTANCE = 0 # 0: min, 1: max, 2: mean

if __name__ == '__main__':
    if DISTANCE == 0:
        bag_l = rosbag.Bag("lidar/lidar_cono_20cm_2024-04-19-12-08-52.bag")
        angle_min = 170
        angle_max = 183
    elif DISTANCE == 1:
        bag_l = rosbag.Bag("lidar/lidar_cono_47cm_2024-04-19-12-10-38.bag")
        angle_min = 170
        angle_max = 179
    elif DISTANCE == 2:
        bag_l = rosbag.Bag("lidar/lidar_cono_120cm_2024-04-19-12-15-58.bag")
        angle_min = 170
        angle_max = 180
    
    rospy.init_node('lidar')
    pub_min = rospy.Publisher('/scan_min', LaserScan, queue_size=10)
    pub_max = rospy.Publisher('/scan_max', LaserScan, queue_size=10)
    pub_mean = rospy.Publisher('/scan_mean', LaserScan, queue_size=10)
    
    rate = rospy.Rate(1)

    angle_min = np.deg2rad(angle_min)
    angle_max = np.deg2rad(angle_max)
    while not rospy.is_shutdown():
        all_scans = []
        for topic, scan, t in bag_l.read_messages(topics=['/scan']):
            start_index = int((angle_min - scan.angle_min) / scan.angle_increment)
            end_index = int((angle_max - scan.angle_min) / scan.angle_increment)

            ranges = scan.ranges[start_index:end_index]
            if np.all(ranges == 0.0):
                continue
            
            all_scans.append(ranges)
            
        all_scans = np.array(all_scans)
        min_distances = np.amin(all_scans, axis=0)
        max_distances = np.amax(all_scans, axis=0)
        mean_distances = np.mean(all_scans, axis=0)
        
        std = np.std(all_scans, axis=0)
        print(f"Mean standard deviation: {np.mean(std):.4f}")
    
        new_scan = LaserScan()
        new_scan.header = scan.header
        new_scan.angle_min = np.deg2rad(angle_min)
        new_scan.angle_max = np.deg2rad(angle_max)
        new_scan.angle_increment = scan.angle_increment
        new_scan.time_increment = scan.time_increment
        new_scan.scan_time = scan.scan_time
        new_scan.range_min = scan.range_min
        new_scan.range_max = scan.range_max
        new_scan.ranges = min_distances
        
        pub_min.publish(new_scan)
        
        new_scan.ranges = max_distances
        pub_max.publish(new_scan)
        
        new_scan.ranges = mean_distances
        pub_mean.publish(new_scan)
        
        rate.sleep()
