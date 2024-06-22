#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Inicializa el nodo de ROS
rospy.init_node('graph_builder')

# Grafo para almacenar las posiciones
graph = nx.Graph()

# Posición actual del robot
current_position = Point()

# Bandera para ir al origen
go_to_origin = False

# Intervalo de tiempo para agregar nodos al grafo (en segundos)
time_interval = rospy.get_param('~time_interval', 5.0)

def odometry_callback(msg):
    global current_position
    current_position = msg.pose.pose.position

# Callback para el tópico /gotostart
def gotostart_callback(msg):
    global go_to_origin
    if msg.data == "GO":
        go_to_origin = True

# Suscribirse al tópico de odometría
odom_sub = rospy.Subscriber("/odom_turtle_bot", Odometry, odometry_callback)

# Suscribirse al tópico /gotostart
gotostart_sub = rospy.Subscriber("/gotostart", String, gotostart_callback)

# Publicador para enviar comandos de velocidad al robot
cmd_pub = rospy.Publisher("/cmd_vel_to_start", Twist, queue_size=10)

def similar_node(graph, node):
    """Determina si hay un nodo con distancia euclidiana menor a 0.15"""
    for n in graph.nodes:
        if (n[0] - node[0])**2 + (n[1] - node[1])**2 < 0.15**2:
            return True
    return False

def add_node_to_graph():
    global graph, current_position, go_to_origin
    while not rospy.is_shutdown():
        x, y = current_position.x, current_position.y
        print("Intentando agregar nodo : (", x, y, ")")
        if (x, y) not in graph:
            if similar_node(graph, (x, y)):
                print("Ya hay un nodo cercano, no se agrega")
            graph.add_node((x, y))
            if len(graph.nodes) > 1:
                last_node = list(graph.nodes)[-2]
                graph.add_edge((x, y), last_node)
            rospy.loginfo(f"Nodo nuevo: {(x, y)}")
            update_graph()
        
        # Revisar si se ha solicitado ir al origen
        if go_to_origin:
            path = find_path_to_origin()
            follow_path(path)
            go_to_origin = False

        rospy.sleep(time_interval)

def find_path_to_origin():
    global graph, current_position
    start = (current_position.x, current_position.y)
    origin = (0, 0)
    if nx.has_path(graph, start, origin):
        path = nx.shortest_path(graph, source=start, target=origin, weight='weight')
        rospy.loginfo(f"Path to origin: {path}")
        return path
    else:
        rospy.logwarn("No path to origin found")
        return []

def follow_path(path):
    if not path:
        return
    
    for node in path:
        target_x, target_y = node
        while not rospy.is_shutdown():
            current_x, current_y = current_position.x, current_position.y
            if abs(current_x - target_x) < 0.1 and abs(current_y - target_y) < 0.1:
                break

        # Calcular el comando de velocidad
        twist = Twist()
        twist.linear.x = 0.1 * (target_x - current_x)
        twist.linear.y = 0.1 * (target_y - current_y)

        # Publicar el comando de velocidad
        cmd_pub.publish(twist)
        rospy.sleep(0.1)

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
    try:
        add_node_to_graph()
        
        # Mantener el nodo activo
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
