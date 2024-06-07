#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from math import pi

COUNT = 0
# Inicializar ROS
rospy.init_node('image_listener', anonymous=True)

# Crear un objeto CvBridge para la conversión entre imágenes ROS y OpenCV
bridge = CvBridge()

def map_value(x, in_min, in_max, out_min, out_max):
    return out_min + (float(x - in_min) / float(in_max - in_min)) * (out_max - out_min)

def image_callback(msg):
    try:
        # Convertir la imagen de ROS a OpenCV
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Convertir de BGR a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #####################################################
    # IDENTIFICAR COLOR AMARILLO

    # Definir el rango de color en HSV
    lower_range = np.array([80, 100, 100])
    upper_range = np.array([100, 255, 255])

    # Umbralizar la imagen HSV para obtener solo los colores en rango
    mask = cv2.inRange(hsv, lower_range, upper_range)

    # Aplicar transformaciones morfológicas para reducir el ruido
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Apertura
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Cierre

    # Encontrar contornos en la máscara
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Suponiendo que hay al menos un contorno, encontrar el contorno más grande
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)

        # Calcular los momentos del contorno más grande
        M = cv2.moments(largest_contour)

        if M["m00"] != 0:
            # Calcular las coordenadas del centro del objeto
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0

        # Dibujar el contorno y el centro del objeto en la imagen original
        cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
        cv2.circle(frame, (cX, cY), 7, (255, 0, 0), -1)
        cv2.putText(frame, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        #print(f"Center: ({cX}, {cY})")

        # el centro es theta = 0 -> -pi/2
        # el inicio (a la izquierda) es (-0.94) + pi/2 = 0.63 = pixel 0
        # el fin (a la derecha) es  (+4.0) +  pi/2 = 5.6     = pixel 1920
        # el medio es 4.71 = - 1.57 = pixel 960
        
        calibration_angle = 4.69 # El frente del robot está en 4.69
        # ahora mapear el pixel a un angulo
        if int(cX) < 960:
            theta = map_value(cX, 0, 960, -0.94, -1.57)
        else:
            theta = map_value(cX, 960, 1920, 4.0, 4.71)
        print(f"Theta: {theta}")

        x = 0.5 * np.cos(theta + np.pi/2)  
        y = 0.5 * np.sin(theta + np.pi/2)
        
        pos_pub.publish(Point(x, y, 0))
    

    # Aplicar AND bit a bit de la máscara y la imagen original
    res = cv2.bitwise_and(frame, frame, mask=mask)

    # Mostrar la imagen con el objeto amarillo resaltado y el centro marcado
    cv2.imshow('image', frame)
    k = cv2.waitKey(10) & 0xFF
    if k == 27:
        rospy.signal_shutdown("User exit")
        cv2.destroyAllWindows()

#def image_callback_pre(msg):
#    global COUNT
#    if COUNT % 10 == 0:
#        image_callback(msg)
#    COUNT += 1

# Suscribirse al tópico de la cámara
image_sub = rospy.Subscriber('usb_cam/image_raw', Image, image_callback)

# Publicar la posición relativa del objeto amarillo
global pos_pub
pos_pub = rospy.Publisher('goal_relative_pos', Point, queue_size=10)

# Mantener el nodo en ejecución
rospy.spin()
