#!/usr/bin/env python3
import random
import time
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

class MainNode:
    def __init__(self):

        self.pub = rospy.Publisher("/cmd_vel",  Twist, queue_size=10)
        self.sub_line = rospy.Subscriber('/line_touch', String, self.turn)
        self.sub_clos = rospy.Subscriber('/close_obj', Twist, self.go_to_goal)
        self.sub_obj = rospy.Subscriber('/object_detected', String, self.update_object)
        self.sub_dist = rospy.Subscriber('/close_obj_dist', Float32, self.update_distance)
        self.sub_start = rospy.Subscriber('/cmd_vel_to_start', Twist, self.go_to_start)
        self.arrived_sub = rospy.Subscriber('/arrived', String, self.arrived_callback)

        self.pub_to_start = rospy.Publisher('/gotostart', String, queue_size=10)
        
        self.state = "EXPLORING"
        self.biggest_object = "NADA"
        self.left = False  # Rueda izquierda está sobre la línea
        self.right = False # Rueda derecha está sobre la línea
        self.last_obj = "NADA"

        # Frecuencia del bucle
        self.rate = rospy.Rate(1)
    
    def arrived_callback(self, data):
        if data.data == "ARRIVED":
            print("Llegamos al origen!!!!")
            self.state = "END"

    def update_distance(self, data):
        if self.state in ["TURNING", "GO_TO_START", "END"]:
            return
        if data.data < 0.22 :
            self.state = "FRENTE_A_OBJETO"
            return
        if data.data < 0.5:
            self.state = "GO_TO_CLOSEST_OBJECT"
            return
        self.state = "EXPLORING"
    
    def update_object(self, data):
        self.biggest_object = data.data
    
    def go_to_start(self,data):
        if self.state == "GO_TO_START":
            self.pub.publish(data)

    def turn(self,data):
        twist = Twist()
        if self.state in ["TURNING","END"]:
            return
        ant_state = self.state
        self.state = "TURNING"
        if data.data == 'Right':
            twist.linear.x = -0.1
            twist.angular.z = 0.3 # Velocidad angular positiva para girar a la izquierda
            self.pub.publish(twist)
            time.sleep(0.5)
            twist.linear.x = 0.1
            self.pub.publish(twist)
            time.sleep(0.5)
        elif data.data == 'Left':
            twist.linear.x = -0.1
            twist.angular.z = -0.3  # Velocidad angular negativa para girar a la derecha
            self.pub.publish(twist)
            time.sleep(0.5)
            twist.linear.x = 0.1
            time.sleep(0.5)
            self.pub.publish(twist)
        elif data.data == 'Both':
            twist.linear.x = -0.05
            twist.angular.z = 0.3
            self.pub.publish(twist)
            time.sleep(0.5)
        self.state = ant_state
    
    def go_to_goal(self,data):
        if self.state == "GO_TO_CLOSEST_OBJECT":
            self.pub.publish(data)

    def stop_robot(self):
        self.pub.publish(Twist())

    def main_loop(self):
        # hago que while flag sea verdadero explore
        while not rospy.is_shutdown():
            print(self.state)
            twist = Twist()
            if self.state in ["END", "TURNING"]:
                continue
            elif self.state == "EXPLORING":
                twist.linear.x = 0.05
                self.pub.publish(twist)
            elif self.state == "FRENTE_A_OBJETO":
                self.stop_robot()
                if self.biggest_object == "roca":
                    # espero a que la quiten
                    print("Esperando a que quiten la roca")
                    self.last_obj = "ROCA"
                    self.stop_robot()
                    continue
                elif self.biggest_object == "minotauro" and self.last_obj != "ROCA" :
                    print("El Minotauro!!")
                    self.stop_robot()
                    self.state = "GO_TO_START"
                    self.pub_to_start.publish("GO")
                    continue
            self.rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('main_node', anonymous=True)
    mn = MainNode()
    try:
        mn.main_loop()
    except rospy.ROSInterruptException:
        pass
