#!/usr/bin/env python3

import math
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import networkx as nx
import threading

# Inicializa el nodo de ROS
rospy.init_node('graph_builder')

# Grafo para almacenar las posiciones
graph = nx.Graph()

# Posición actual del robot
current_position = Point()

# Intervalo de tiempo para agregar nodos al grafo (en segundos)
time_interval = rospy.get_param('~time_interval', 5.0)

# Lock para asegurar la coherencia en el acceso a la posición actual y al grafo
lock = threading.Lock()

def odometry_callback(msg):
    global current_position
    with lock:
        current_position = msg.pose.pose.position


# Suscribirse al tópico de odometría
odom_sub = rospy.Subscriber("/odom_turtle", Odometry, odometry_callback)

def add_node_to_graph():
    global graph, current_position
    while not rospy.is_shutdown():
        with lock:
            print(current_position)
            x, y = current_position.x, current_position.y
            if (x, y) not in graph:
                graph.add_node((x, y))
                if len(graph.nodes) > 1:
                    last_node = list(graph.nodes)[-2]
                    graph.add_edge((x, y), last_node)
                rospy.loginfo(f"Added node: {(x, y)}")

        rospy.sleep(time_interval)

def euclidean_distance(node1, node2):
    return math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)

def optimize_graph(the_graph):
    """
        si la distancia euclidiana entre dos nodos es menor a 0.1 metros, se fusionan
    """
    for node in the_graph.nodes:
        for neighbor in the_graph.nodes - {node}:
            if euclidean_distance(node, neighbor) < 0.1:
                the_graph = nx.contracted_nodes(the_graph, node, neighbor, self_loops=False)
                rospy.loginfo(f"Contracted nodes: {node}, {neighbor}")
        rospy.sleep(time_interval)

# TODO: usar esta función
def find_path_to_origin():
    global graph, current_position
    with lock:
        optimize_graph(graph)
        start = (current_position.x, current_position.y)
        origin = (0, 0)
        if nx.has_path(graph, start, origin):
            path = nx.shortest_path(graph, source=start, target=origin, weight='weight')
            rospy.loginfo(f"Path to origin: {path}")
            return path
        else:
            rospy.logwarn("No path to origin found")
            return []

if __name__ == '__main__':
    try:
        # Thread para agregar nodos al grafo
        graph_thread = threading.Thread(target=add_node_to_graph)
        graph_thread.start()
        
        # Mantener el nodo activo
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
