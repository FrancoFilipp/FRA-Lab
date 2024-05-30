#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32  # Asumiendo que el mensaje es de tipo Float32

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " heard %s", data.data)

def listener():
    rospy.init_node('sensor_listener', anonymous=True)
    rospy.Subscriber("adc", Float32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
