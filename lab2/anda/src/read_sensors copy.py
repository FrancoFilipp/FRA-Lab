#!/usr/bin/env python3
import rospy
from rosserial_arduino.msg import Adc
from std_msgs.msg import Float32

class ReadSensors:

    def __init__(self):
        rospy.init_node('publish_sensor_data', anonymous=True)

        self.topic_publisher = rospy.Publisher('/my_topico/ir_pasabajos_001', Float32, queue_size=10)
        self.topic_publisher_1 = rospy.Publisher('/my_topico/ir_pasabajos_05', Float32, queue_size=10)

        self.pose_subscriber = rospy.Subscriber('/adc', Adc, self.repost_data) 

        self.o_ant = -1
        self.o_ant_1 = -1

    def repost_data(self, data):
        o_actual = int(data.adc0)
        if self.o_ant < 0:
            self.o_ant = o_actual

        # hago filtrado pasa bajos con alpha = 0.01
        out = self.o_ant + 0.01 * (o_actual - self.o_ant)
        #print(out)
        self.o_ant = out
        self.topic_publisher.publish(out)

        # hago filtrado pasa bajos con alpha = 0.5

        if self.o_ant_1 < 0:
            self.o_ant_1 = o_actual
        
        out_1 = self.o_ant + 0.5 * (o_actual - self.o_ant)
        self.o_ant_1 = out_1
        self.topic_publisher_1.publish(out_1)

if __name__ == '__main__':

    ReadSensors()
    # Mantener el nodo en ejecución hasta que se detenga manualmente.
    rospy.spin()
