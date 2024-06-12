#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MainNode:
    def __init__(self):
        self.flag = False
        self.sub_ = rospy.Subscriber('/line_touch', String, self.turn)
        self.sub = rospy.Subscriber('/close_obj', Twist, self.go_to_goal) # Importar rosserial_arduino.msg.ADC
        self.pub = rospy.Publisher("/dynamixel_workbench/cmd_vel",  Twist, queue_size=10)
        self.left = False  # Rueda izquierda está sobre la línea
        self.right = False # Rueda derecha está sobre la línea

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
        self.pub.publish(data)

    def dodge_line(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('main_node', anonymous=True)
    ld = MainNode()
    try:
        ld.dodge_line()
    except rospy.ROSInterruptException:
        pass
