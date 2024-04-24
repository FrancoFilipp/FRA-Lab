#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np

def callback(scan):
    min_distance = float('inf')
    min_index = -1

    for i, distance in enumerate(scan.ranges):
        if distance < min_distance and distance > scan.range_min and distance < scan.range_max:
            min_distance = distance
            min_index = i

    angle_of_closest_object = scan.angle_min + min_index * scan.angle_increment
    direction_clocwise = False
    
    if (angle_of_closest_object > np.pi):
        angle_of_closest_object = 2 * np.pi - angle_of_closest_object
        direction_clocwise = True
    
    angle_pub.publish(angle_of_closest_object)
    direction_clockwise.publish(direction_clocwise)

    # Log the closest distance and its angle
    print(f"Distance: {min_distance: .2f} meters, Angle: {angle_of_closest_object: .2f} radians, Clockwise: {clockwise}")

def listener():
    rospy.init_node('closest_object')
    global angle_pub
    angle_pub = rospy.Publisher('rotation_angle', Float64, queue_size=10)
    direction_pub = rospy.Publisher('direction_clockwise', Bool, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

