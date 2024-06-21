#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np
from math import pi

def callback(scan):
    min_distance = float('inf')
    min_index = -1

    for i, distance in enumerate(scan.ranges):
        if distance < min_distance and distance > scan.range_min and distance < scan.range_max:
            min_distance = distance
            min_index = i

    theta = scan.angle_min + min_index * scan.angle_increment
    print("Theta: ", theta)
    
    calibration_angle = pi/2#4.69 # El frente del robot está en 4.69
    theta -= calibration_angle
    
    if min_index == -1:
        x = 0
        y = 0
    else:
        # TODO: Verificar que ande bien
        x = min_distance * np.cos(theta + np.pi/2)  
        y = min_distance * np.sin(theta + np.pi/2)
        
    pos_pub.publish(Point(x, y, 0))
    
    # Log the closest distance and its angle
    print(f"Distance: {min_distance: .2f} meters, Angle: {theta: .2f} radians")

def listener():
    rospy.init_node('closest_object')
    global pos_pub
    pos_pub = rospy.Publisher('goal_relative_pos', Point, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

