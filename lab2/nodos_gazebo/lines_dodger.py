#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from rosserial_arduino.msg import ADC

class LineDodger:
    def __init__(self):
        self.pub = rospy.Publisher('/line_touch', String, queue_size=10)
        self.sub = rospy.Subscriber('/adc', rosserial_arduino.msg.ADC, self.callback) # Importar rosserial_arduino.msg.ADC
        self.left = False  # Rueda izquierda está sobre la línea?
        self.right = False # Rueda derecha está sobre la línea?

    def callback(self, data):
        # TODO: Verificar el acceso a data y cual es left y cual es right
        self.left = data.adc0 < 900
        self.right = data.adc1 < 530
        self.publish_line_touch()

    def publish_line_touch(self):
        if self.left:
            self.pub.publish("Left")
        elif self.right:
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
