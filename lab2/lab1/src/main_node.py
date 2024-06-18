#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist

class MainNode:
    def __init__(self):
        self.sub_line = rospy.Subscriber('/line_touch', String, self.turn)
        self.sub_clos = rospy.Subscriber('/close_obj', Twist, self.go_to_goal)
        self.sub_obj = rospy.Subscriber('/object_detected', String, self.update_object)
        self.sub_dist = rospy.Subscriber('/close_obj_dist', Int32, self.update_distance)
        self.sub_start = rospy.Subscriber('/cmd_vel_to_start', Twist, self.go_to_start)

        self.pub_to_start = rospy.Publisher('/gotostart', String, queue_size=10)
        self.pub = rospy.Publisher("/dynamixel_workbench/cmd_vel",  Twist, queue_size=10)
        
        self.state = "EXPLORING"
        self.biggest_object = "NADA"
        self.left = False  # Rueda izquierda está sobre la línea
        self.right = False # Rueda derecha está sobre la línea
        self.closest_distance = 99999999

        # Frecuencia del bucle
        self.rate = rospy.Rate(10)

    def update_distance(self, data):
        if data < 0.15 :
            self.state = "FRENTE_A_OBJETO"
            return
        self.state = "EXPLORING"
    
    def update_object(self, data):
        self.biggest_object = data
    
    def go_to_start(self,data):
        if self.state == "GO_TO_START":
            self.pub.publish(data)

    def turn(self,data):
        twist = Twist()
        if data.data == 'Right':
            print("Right")
            # Giro a la izquierda
            twist.angular.z = 0.7  # Velocidad angular positiva para girar a la izquierda
            self.pub.publish(twist)
        elif data.data == 'Left':
            # Giro a la derecha
            print("Left")
            twist.angular.z = -0.7  # Velocidad angular negativa para girar a la derecha
            self.pub.publish(twist)
        elif data.data == 'Both':
            # Retroceder
            print("Both")
            twist.linear.x = -0.08
            twist.angular.z = 0.7
            self.pub.publish(twist)
    
    def go_to_goal(self,data):
        if self.state == "GO_TO_CLOSEST_OBJECT":
            self.pub.publish(data)

    def stop_robot(self):
        self.pub.publish(Twist())

    def main_loop(self):
        twist = Twist()
        # hago que while flag sea verdadero explore
        while not rospy.is_shutdown():
            if self.state == "EXPLORING":
                twist.linear.x = 0.5
                self.pub.publish(twist)
            if self.state == "FRENTE_A_OBJETO":
                if self.biggest_object == "roca":
                    # espero a que la quiten
                    self.stop_robot()
                    continue
                if self.biggest_object == "minotauro":
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
