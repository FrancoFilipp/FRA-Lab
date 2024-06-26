#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# Inicializa el nodo de ROS
rospy.init_node('odometry_publisher')

# Publicador del tópico odom
odom_pub = rospy.Publisher("/odom_turtle_bot", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

# Variables de posición y orientación
x = 0.0
y = 0.0
th = 0.0

# Variables de velocidad inicializadas a cero
vx = 0.0
vy = 0.0
vth = 0.0

# Función callback para actualizar las velocidades desde el tópico cmd_vel
def cmd_vel_callback(msg):
    global vx, vy, vth
    vx = msg.linear.x
    vy = msg.linear.y
    vth = msg.angular.z
    ## print(f"Velocidades: vx={vx}, vy={vy}, vth={vth}")

# Suscribirse al tópico cmd_vel
rospy.Subscriber("/dynamixel_workbench/cmd_vel", Twist, cmd_vel_callback)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(50.0)  # Frecuencia de 10 Hz
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # Calcula la odometría de manera típica dada las velocidades del robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # Ya que toda la odometría es 6DOF, necesitaremos un quaternion creado desde el yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # Primero, publicamos la transformación sobre tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # Luego, publicamos el mensaje de odometría sobre ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # Configura la posición
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    print(f"x={x}, y={y}, th={th}")

    # Configura la velocidad
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # Publica el mensaje
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
