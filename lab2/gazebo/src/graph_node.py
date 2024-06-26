#!/usr/bin/env python3

from math import atan2
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

# Bandera para ir al origen
go_to_origin = False

# Intervalo de tiempo para agregar nodos al grafo (en segundos)
#time_interval = rospy.get_param('~time_interval', 5.0)

def odometry_callback(msg):
    global current_position
    current_position = msg.pose.pose.position

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
    rate = rospy.Rate(1/5)
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
                    graph.add_edge(last_node, similar_n)
                else:
                    graph.add_node((x, y))
                    if len(graph.nodes) > 1:
                        last_node = list(graph.nodes)[-2]
                        graph.add_edge((x, y), last_node)
                    
                    rospy.loginfo(f"Nodo nuevo: {(x, y)}")
                update_graph()
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
    
def linear_vel(euclidean_distance, constant=1.5):
    max_vel = 0.05
    return min(constant * euclidean_distance, max_vel)

def steering_angle(x,y):
    return atan2(y, x)

def angular_vel(x,y,constant=0.5):
    """
    TODO: Chequear esto, supongo que el theta es hacia a donde apunta el frente del robot,
    que creo que es theta=0. En este caso el robot siempre estaría "fijo" en el origen aputando
    hacia arriba, y pensamos como que lo que se mueve es el punto de destino.
    """
    theta = 0
    return constant * (steering_angle(x,y) - theta)

def follow_path(path):
    if not path:
        print("No path to follow")
        return
    rate = rospy.Rate(10)
    for node in path:
        print("Siguiente nodo: ", node)
        target_x, target_y = node
        while not rospy.is_shutdown():
            current_x, current_y = current_position.x, current_position.y
            # si la distancia euclidea es menor a 0.05
            euclidean_distance = ((target_x - current_x)**2 + (target_y - current_y)**2)**0.5
            print("Distancia euclidiana al nodo objetivo: ", euclidean_distance)
            if euclidean_distance < 0.2:
                break
            # Calcular el comando de velocidad
            twist = Twist()
            twist.linear.x = linear_vel(euclidean_distance)
            twist.angular.z = angular_vel(target_x - current_x, target_y - current_y)

            # Publicar el comando de velocidad
            cmd_pub.publish(twist)
            #rospy.sleep(0.1)
            rate.sleep()

    # Detener el robot al final
    cmd_pub.publish(Twist())

# Función para actualizar la visualización del grafo
def update_graph():
    global graph
    plt.clf()
    pos = {node: node for node in graph.nodes()}
    nx.draw(graph, pos, with_labels=True, node_size=50, font_size=8)
    plt.draw()
    plt.savefig("/home/franco/FRA-Lab/graph_imgs/{0}.png".format(len(graph.nodes())))

if __name__ == '__main__':
    # Inicializa el nodo de ROS
    rospy.init_node('graph_builder')

    # Suscribirse al tópico de odometría
    odom_sub = rospy.Subscriber("/odom_turtle_bot", Odometry, odometry_callback)

    # Suscribirse al tópico /gotostart
    gotostart_sub = rospy.Subscriber("/gotostart", String, gotostart_callback)

    # Publicador para enviar comandos de velocidad al robot
    cmd_pub = rospy.Publisher("/cmd_vel_to_start", Twist, queue_size=10)
    
    try:
        add_node_to_graph()
        # Revisar si se ha solicitado ir al origen
        if go_to_origin:
            print("Yendo al origen")
            start_node = closest_node(graph, current_position)
            path = find_path_to_origin(start_node)
            follow_path(path)
            #go_to_origin = False
        #rospy.sleep(time_interval)
        # Mantener el nodo activo
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
