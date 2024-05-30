#!/usr/bin/env python3
import rospy
from rosserial_arduino.msg import Adc

#asigno filtro pasabajo
pasabajo_alpha = 0.5

# funcion de cálculo psabajo
def pasabajo(alpha,sensor_read):
    out_old = 0
    out_new = out_old + alpha*(sensor_read-out_old)
    out_old = out_new
    return out_new

#funcion callback se subscriber
def callback_publish_pasabajo(msg:Adc):
    msg_out = Adc()
    
    valor_pasabajo = pasabajo(pasabajo_alpha,msg.adc0)
    
    msg_out.adc0 = int(valor_pasabajo)
    msg_out.adc1 = 0
    msg_out.adc2 = 0
    msg_out.adc3 = 0
    msg_out.adc4 = 0
    msg_out.adc5 = 0

    rospy.loginfo("valor crudo " + str(msg.adc0))
    rospy.loginfo("valor pasabajo " + str(valor_pasabajo) )
    
    pub.publish(msg_out)


if __name__ == '__main__':
    rospy.init_node("filtro_pasabajo")
    rospy.loginfo("node started")
    pub = rospy.Publisher("/my_topic/adc_filtrado",Adc,queue_size=10)
    sub = rospy.Subscriber("/adc", Adc, callback = callback_publish_pasabajo)

    rospy.loginfo("node started")

    rospy.spin()
