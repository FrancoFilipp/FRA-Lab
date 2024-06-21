#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

COUNT = 0
# Inicializar ROS
rospy.init_node('image_listener', anonymous=True)

# Crear un objeto CvBridge para la conversión entre imágenes ROS y OpenCV
bridge = CvBridge()

def image_callback(msg):
    try:
        # Convertir la imagen de ROS a OpenCV
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Convertir de BGR a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # IDENTIFICACIÓN DE COLORES
    yellow_lower_range = np.array([10, 100, 100])
    yellow_upper_range = np.array([25, 255, 255])

    blue_lower_range = np.array([67, 10, 10])
    blue_upper_range = np.array([202, 255, 255])

    green_lower_range = np.array([35, 10, 10])
    green_upper_range = np.array([80, 255, 255])

    # Primer rango para tonos de rojo más bajos
    lower_red_1 = np.array([0, 100, 100])
    upper_red_1 = np.array([10, 255, 255])

    # Segundo rango para tonos de rojo más altos
    lower_red_2 = np.array([160, 100, 100])
    upper_red_2 = np.array([179, 255, 255])

    # Umbralizar la imagen HSV para obtener solo los colores en rango
    yellow_mask = cv2.inRange(hsv, yellow_lower_range, yellow_upper_range)
    blue_mask = cv2.inRange(hsv, blue_lower_range, blue_upper_range)
    green_mask = cv2.inRange(hsv, green_lower_range, green_upper_range)
    red_mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    red_mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    # Aplicar transformaciones morfológicas para reducir el ruido
    kernel = np.ones((10, 10), np.uint8)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)  # Apertura
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)  # Cierre
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)  # Apertura
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)  # Cierre
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)  # Apertura
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)  # Cierre
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)  # Apertura
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)  # Cierre


    # Encontrar contornos en la máscara
    y_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    b_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    g_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    r_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # unir contornos
    minotauro_contours = y_contours + b_contours
    rocas_contours = g_contours + r_contours

# Suponiendo que hay al menos un contorno
    if len(minotauro_contours) > 0 or len(rocas_contours) > 0:
        largest_minotauro = None
        largest_rocas = None
        largest_object = None

        if len(minotauro_contours) > 0:
            largest_minotauro = max(minotauro_contours, key=cv2.contourArea)

        if len(rocas_contours) > 0:
            largest_rocas = max(rocas_contours, key=cv2.contourArea)

        if largest_minotauro is not None and largest_rocas is not None:
            largest_object = max([largest_minotauro, largest_rocas], key=cv2.contourArea)
        elif largest_minotauro is not None:
            largest_object = largest_minotauro
        elif largest_rocas is not None:
            largest_object = largest_rocas

        if largest_object is not None:
            # Calcular los momentos del contorno más grande
            M = cv2.moments(largest_object)

            if M["m00"] != 0:
                # Calcular las coordenadas del centro del objeto
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            # Dibujar el contorno y el centro del objeto en la imagen original
            cv2.drawContours(frame, [largest_object], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cX, cY), 7, (255, 0, 0), -1)
            cv2.putText(frame, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            if largest_minotauro is not None and (largest_rocas is None or cv2.contourArea(largest_minotauro) > cv2.contourArea(largest_rocas)):
                print("Objeto mas cercano: Minotauro")
                # pub.publish("minotauro")
            else:
                print("Objeto mas cercano: Roca")
                # pub.publish("roca")
    else:
        print("No se detectaron ni minotauros ni rocas")


    # Aplicar AND bit a bit de la máscara y la imagen original
    #res = cv2.bitwise_and(frame, frame, mask=mask)

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
image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, image_callback)

# Mantener el nodo en ejecución
rospy.spin()
