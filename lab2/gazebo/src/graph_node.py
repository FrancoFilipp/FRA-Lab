#!/usr/bin/env python3

from math import atan2
import math
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation



# Grafo para almacenar las posiciones
graph = nx.Graph()

# Posición actual del robot
current_position = Point()

# Parámetros del controlador
Kp_linear = 0.5
Kp_angular = 1.0

# Bandera para ir al origen
go_to_origin = False

# Intervalo de tiempo para agregar nodos al grafo (en segundos)
#time_interval = rospy.get_param('~time_interval', 5.0)

def get_yaw_from_quaternion(q):
        """
        Convertir un quaternion en ángulo de Euler (yaw).
        """
        quaternion = (q.x, q.y, q.z, q.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

def odometry_callback(msg):
    global current_position, th
    current_position = msg.pose.pose.position
    orientation_q = msg.pose.pose.orientation
    th = get_yaw_from_quaternion(orientation_q)

# Callback para el tópico /gotostart
def gotostart_callback(msg):
    global go_to_origin
    if msg.data == "GO":
        print("Recibida orden de ir al origen")
        go_to_origin = True

def similar_node(graph, node):
    """Determina si hay un nodo con distancia euclidiana menor a 0.15"""
    for n in graph.nodes:
        if (n[0] - node[0])**2 + (n[1] - node[1])**2 < 0.15**2:
            return n
    return False

def closest_node(graph, position):
    """Encuentra el nodo más cercano a la posición actual"""
    min_dist = 999999
    closest = None
    for node in graph.nodes:
        dist = (node[0] - position.x)**2 + (node[1] - position.y)**2
        if dist < min_dist:
            min_dist = dist
            closest = node
    return closest

def add_node_to_graph():
    rate = rospy.Rate(1/8)
    global graph, current_position, go_to_origin
    while not rospy.is_shutdown() and not go_to_origin:
        if not go_to_origin:
            x, y = current_position.x, current_position.y
            print("Intentando agregar nodo : (", x, y, ")")
            if (x, y) not in graph:
                similar_n = similar_node(graph, (x, y))
                if len(graph.nodes) > 0 and similar_n == list(graph.nodes)[-1]:
                    # No hacer nada si es el último nodo agregado
                    continue
                elif len(graph.nodes) > 0 and similar_n:
                    # Unir el nodo last_node con similar_node
                    last_node = list(graph.nodes)[-1]
                    distance = math.sqrt((last_node[0] - similar_n[0])**2 + (last_node[1] - similar_n[1])**2)
                    graph.add_edge(last_node, similar_n, weight=distance)
                else:
                    graph.add_node((x, y))
                    if len(graph.nodes) > 1:
                        last_node = list(graph.nodes)[-2]
                        distance = math.sqrt((x - last_node[0])**2 + (y - last_node[1])**2)
                        graph.add_edge((x, y), last_node, weight=distance)
                    
                    rospy.loginfo(f"Nodo nuevo: {(x, y)}")
        rate.sleep()

def find_path_to_origin(start):
    global graph
    origin = (0, 0)
    if nx.has_path(graph, start, origin):
        path = nx.shortest_path(graph, source=start, target=origin, weight='weight')
        rospy.loginfo(f"Path to origin: {path}")
        return path
    else:
        rospy.logwarn("No path to origin found")
        print("No path to origin found")
        return []
    
def follow_path(path):
    global current_position, th
    if not path:
        print("No path to follow")
        return
    rate = rospy.Rate(1)
    for node in path:
        print("Siguiente nodo: ", node)
        target_x, target_y = node
        while not rospy.is_shutdown():
            current_x, current_y = current_position.x, current_position.y

            # Calcular errores de posición y orientación
            error_x = target_x - current_x
            error_y = target_y - current_y
            distance = math.sqrt(error_x**2 + error_y**2)
            target_angle = math.atan2(error_y, error_x)
            error_th = target_angle - th

            # Normalizar el error de orientación
            error_th = math.atan2(math.sin(error_th), math.cos(error_th))

            # Calcular velocidades de control
            linear_speed = Kp_linear * distance
            angular_speed = Kp_angular * error_th

            # Limitar las velocidades para evitar movimientos bruscos
            linear_speed = max(min(linear_speed, 0.08), -0.08)
            angular_speed = max(min(angular_speed, 0.3), -0.3)

            # Publicar las velocidades calculadas
            cmd_vel = Twist()
            cmd_vel.linear.x = linear_speed
            cmd_vel.angular.z = angular_speed
            cmd_pub.publish(cmd_vel)
            #rospy.sleep(0.1)

            # Verificar si hemos llegado al punto de inicio
            if distance < 0.1:
                break
            rate.sleep()

    # Detener el robot al final
    cmd_pub.publish(Twist())
    arrived_pub.publish("ARRIVED")

# Función para actualizar la visualización del grafo
def save_graph(path):
    global graph
    plt.clf()
    pos = nx.spring_layout(graph)
    nx.draw(graph, pos, with_labels=True, node_size=500, node_color='skyblue')
    nx.draw_networkx_nodes(graph, pos, nodelist=path, node_color='red', node_size=500)
    plt.savefig("/home/franco/FRA-Lab/graph_imgs/graph.png")
    

if __name__ == '__main__':
    # Inicializa el nodo de ROS
    rospy.init_node('graph_builder')

    # Suscribirse al tópico de odometría
    odom_sub = rospy.Subscriber("/odom_turtle_bot", Odometry, odometry_callback)

    # Suscribirse al tópico /gotostart
    gotostart_sub = rospy.Subscriber("/gotostart", String, gotostart_callback)

    # Publicador para enviar comandos de velocidad al robot
    cmd_pub = rospy.Publisher("/cmd_vel_to_start", Twist, queue_size=10)

    # Publicar cuando se llego al origen
    arrived_pub = rospy.Publisher("/arrived", String, queue_size=10)
    
    try:
        add_node_to_graph()
        # Revisar si se ha solicitado ir al origen
        if go_to_origin:

            print("Yendo al origen")
            
            start_node = closest_node(graph, current_position)
            
            path = find_path_to_origin(start_node)

            #guardamos el grafo en png con el path a seguir
            save_graph(path)
            
            follow_path(path)
            
            go_to_origin = False
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
