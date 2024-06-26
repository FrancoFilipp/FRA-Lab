#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point
from math import pow, atan2, sqrt
from turtlesim.srv import SetPen
import random
import time

# TENER EN CUENTA: La posición del robot es siempre el origen y apunta hacia arriba

class Robot:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/close_obj', Twist, queue_size=10)

        self.distance_publisher = rospy.Publisher('/close_obj_dist', Float32, queue_size=10)
        
        self.goal_position_subscriber = rospy.Subscriber('/goal_relative_pos', Point, self.update_goal)

        self.rate = rospy.Rate(1)
        self.goal_pos = Point()

    def update_goal(self, data):
        self.goal_pos = data

    def euclidean_distance(self):
        return sqrt(pow(self.goal_pos.x, 2) + pow(self.goal_pos.y, 2))

    def linear_vel(self, constant=1.5):
        max_vel = 0.1
        return min(constant * self.euclidean_distance(), max_vel)

    def steering_angle(self):
        return atan2(self.goal_pos.y, self.goal_pos.x)

    def angular_vel(self, constant=0.7):
        """
        TODO: Chequear esto, supongo que el theta es hacia a donde apunta el frente del robot,
        que creo que es theta=0. En este caso el robot siempre estaría "fijo" en el origen aputando
        hacia arriba, y pensamos como que lo que se mueve es el punto de destino.
        """
        theta = 0
        return constant * (self.steering_angle() - theta)

    def move2goal(self):
        vel_msg = Twist()
        
        rate = rospy.Rate(0.2) # Cuando encuentra el objetivo, espera 5 segundos y luego vuelve a buscar
        
        distance_tolerance = 0.17
        while not rospy.is_shutdown():
            while self.euclidean_distance() >= distance_tolerance and not rospy.is_shutdown():
                self.distance_publisher.publish(self.euclidean_distance())
                
                vel_msg.linear.x = self.linear_vel()
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel()

                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()

            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            
            rate.sleep()
       
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        x = Robot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
