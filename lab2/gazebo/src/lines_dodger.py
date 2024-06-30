#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from rosserial_arduino.msg import Adc
import time

class LineDodger:
    def __init__(self):
        self.pub = rospy.Publisher('/line_touch', String, queue_size=0)
        self.sub = rospy.Subscriber('/adc', Adc, self.callback)
        self.left = False  # Rueda izquierda está sobre la línea?
        self.right = False # Rueda derecha está sobre la línea?
        self.t0 = time.time() - 0.4

    def callback(self, data):
        self.left = data.adc0 > 500
        self.right = data.adc1 > 500
        self.publish_line_touch()

    def publish_line_touch(self):
        if time.time() - self.t0 > 0.4:
            self.t0 = time.time()
        else:
            return
        
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
