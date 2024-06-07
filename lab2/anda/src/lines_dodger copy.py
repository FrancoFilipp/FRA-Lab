#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from rosserial_arduino.msg import Adc
import time

class LineDodger:
    def __init__(self):
        self.pub = rospy.Publisher('/line_touch', String, queue_size=10)
        self.sub = rospy.Subscriber('/adc', Adc, self.callback) # Importar rosserial_arduino.msg.ADC
        self.left = False  # Rueda izquierda está sobre la línea?
        self.right = False # Rueda derecha está sobre la línea?
        self.clock_start = time.time()

    def callback(self, data):
        # TODO: Verificar el acceso a data y cual es left y cual es right
        self.left = data.adc1 < 600
        self.right = data.adc0 < 900
        self.publish_line_touch()

    def publish_line_touch(self):
        clock_end = time.time()
        #if clock_end - self.clock_start < 0.5:
        #    return
        
        self.clock_start = clock_end
        if self.left and self.right:
            self.pub.publish("Both")
        else:
            if self.left:
                self.pub.publish("Left")
            
            if self.right:
                self.pub.publish("Right")

    def dodge_line(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('line_dodger', anonymous=True)
    ld = LineDodger()
    try:
        ld.dodge_line()
    except rospy.ROSInterruptException:
        pass
