#!/usr/bin/env python3
import rospy
from rosserial_arduino.msg import Adc
from std_msgs.msg import Float32
"""
Implemente un nodo que reciba una posicion objetivo en un tópico y controle a 
la tortuga (a traves de mensajes Twist al tópico cmd_vel) del paquete turtlesim
para alcanzarlo. Cada vez que se recibe una nueva posición objetivo se debe
cambiar el color del trazo-
"""

class ReadSensors:

    def __init__(self):
        rospy.init_node('publish_sensor_data', anonymous=True)

        # Publisher al tópico /turtle1/cmd_vel para enviar mensajes de control.
        self.topic_publisher = rospy.Publisher('/my_topico/ir_sensor', Float32, queue_size=10)

        # Suscribirse al tópico /turtle1/pose para obtener la posición actual de la tortuga.
        self.pose_subscriber = rospy.Subscriber('/adc', Adc, self.repost_data) 

        self.o_ant = -1
        self.alpha = 0.01
        #self.alpha = 0.5

        #self.rate = rospy.Rate(10)

    def repost_data(self, data):
        o_actual = int(data.adc0)
        if self.o_ant < 0:
            self.o_ant = o_actual

        # hago filtrado pasa bajos
        out = self.o_ant + self.alpha * (o_actual - self.o_ant)
        #print(out)
        self.o_ant = out

        self.topic_publisher.publish(out)

        #self.rate.sleep()

if __name__ == '__main__':

    ReadSensors()
    # Mantener el nodo en ejecución hasta que se detenga manualmente.
    rospy.spin()