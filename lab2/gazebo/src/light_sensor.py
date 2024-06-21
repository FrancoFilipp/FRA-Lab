#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rosserial_arduino.msg import Adc

# Inicializar ROS
rospy.init_node('light_sensor_simulator', anonymous=True)

# Crear un objeto CvBridge para la conversión entre imágenes ROS y OpenCV
bridge = CvBridge()

# Variables globales para almacenar los valores de brillo total de cada cámara
total_value_izq = 0
total_value_der = 0

def image_callback_izq(msg):
    global total_value_izq
    try:
        # Convertir la imagen de ROS a OpenCV
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame, (50, 50))
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        total_value_izq = cv2.sumElems(gray_image)[0]
        publish_adc_values()

    except CvBridgeError as e:
        print(e)
        return

def image_callback_der(msg):
    global total_value_der
    try:
        # Convertir la imagen de ROS a OpenCV
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame, (50, 50))
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        total_value_der = cv2.sumElems(gray_image)[0]
        publish_adc_values()

    except CvBridgeError as e:
        print(e)
        return

def publish_adc_values():
    global total_value_izq, total_value_der
    simed_adc = Adc()
    simed_adc.adc0 = np.uint16(total_value_izq / 600)
    simed_adc.adc1 = np.uint16(total_value_der / 600)
    simed_adc.adc2 = 0
    simed_adc.adc3 = 0
    simed_adc.adc4 = 0
    simed_adc.adc5 = 0
    
    print("Izq: ", total_value_izq, "Der: ", total_value_der)
    sim_adc_pub.publish(simed_adc)

# Suscribirse al tópico de la cámara
image_izq = rospy.Subscriber('camera1/rgb/image_raw', Image, image_callback_izq)
image_der = rospy.Subscriber('camera2/rgb/image_raw', Image, image_callback_der)

# Crear el publicador para el tópico 'adc'
sim_adc_pub = rospy.Publisher('adc', Adc, queue_size=10)

# Mantener el nodo en ejecución
rospy.spin()