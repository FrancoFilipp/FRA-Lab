#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rosserial_arduino.msg import Adc
from std_msgs.msg import Float32
import numpy as np

# Inicializar ROS
rospy.init_node('light_sensor_simulator', anonymous=True)


# Crear un objeto CvBridge para la conversión entre imágenes ROS y OpenCV
bridge = CvBridge()

def image_callback(msg):
    try:
        # Convertir la imagen de ROS a OpenCV
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame,(50,50))
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        total_value = cv2.sumElems(gray_image)[0]
        
        simed_adc = Adc()
        simed_adc.adc0 = np.uint16(total_value/600)
        simed_adc.adc1 = 1
        simed_adc.adc2 = 0
        simed_adc.adc3 = 0
        simed_adc.adc5 = 0
        simed_adc.adc4 = 0
        

        sim_adc_pub.publish(simed_adc)


    except CvBridgeError as e:
        print(e)
        return


    # Mostrar la imagen con el objeto amarillo resaltado y el centro marcado
    cv2.imshow('image', frame)
    k = cv2.waitKey(10) & 0xFF
    if k == 27:
        rospy.signal_shutdown("User exit")
        cv2.destroyAllWindows()

# Suscribirse al tópico de la cámara
image_sub = rospy.Subscriber('camera1/rgb/image_raw', Image, image_callback)
sim_adc_pub = rospy.Publisher('simAdc', Adc, queue_size=10)

#  Mantener el nodo en ejecución
rospy.spin()
