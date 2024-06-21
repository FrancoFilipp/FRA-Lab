#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import tf
import math

class GoToStart:
    def __init__(self):
        rospy.init_node('go_to_start_node', anonymous=True)

        # Suscribirse a los tópicos /odom_turtle y /gotostart
        self.odom_sub = rospy.Subscriber('/odom_turtle', Odometry, self.odom_callback)
        self.gotostart_sub = rospy.Subscriber('/gotostart', String, self.gotostart_callback)

        # Publicador del tópico /cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_to_start', Twist, queue_size=10)

        # Inicializar la posición y orientación del robot
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Inicializar bandera de retorno
        self.go_to_start = False

        # Parámetros del controlador
        self.Kp_linear = 0.5
        self.Kp_angular = 1.0

        # Frecuencia del bucle
        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        # Actualizar la posición y orientación del robot
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        self.th = self.get_yaw_from_quaternion(orientation_q)

    def get_yaw_from_quaternion(self, q):
        """
        Convertir un quaternion en ángulo de Euler (yaw).
        """
        quaternion = (q.x, q.y, q.z, q.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def gotostart_callback(self, msg):
        # Si se recibe el mensaje "GO", activar el retorno al punto (0, 0)
        if msg.data == "GO":
            self.go_to_start = True

    def control_to_start(self):
        """
        Controlador proporcional para mover el robot al punto (0, 0).
        """
        if not self.go_to_start:
            return

        # Calcular errores de posición y orientación
        error_x = 0.0 - self.x
        error_y = 0.0 - self.y
        distance = math.sqrt(error_x**2 + error_y**2)
        target_angle = math.atan2(error_y, error_x)
        error_th = target_angle - self.th

        # Normalizar el error de orientación
        error_th = math.atan2(math.sin(error_th), math.cos(error_th))

        # Calcular velocidades de control
        linear_speed = self.Kp_linear * distance
        angular_speed = self.Kp_angular * error_th

        # Limitar las velocidades para evitar movimientos bruscos
        linear_speed = max(min(linear_speed, 0.1), -0.1)
        angular_speed = max(min(angular_speed, 0.5), -0.5)

        # Publicar las velocidades calculadas
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_speed
        cmd_vel.angular.z = angular_speed
        self.cmd_vel_pub.publish(cmd_vel)

        # Verificar si hemos llegado al punto de inicio
        if distance < 0.1 and abs(error_th) < 0.1:
            self.go_to_start = False  # Desactivar el retorno
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        while not rospy.is_shutdown():
            self.control_to_start()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = GoToStart()
        node.run()
    except rospy.ROSInterruptException:
        pass
