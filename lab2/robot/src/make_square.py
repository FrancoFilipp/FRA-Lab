#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from math import pi as PI
import time

speed = 0.2

def handle_move_square(s, r):
	# para mover el robot cambiar el topico /turtle1/cmd_vel por el topico /dynamixel_workbench/cmd_vel
	pub = rospy.Publisher("/dynamixel_workbench/cmd_vel", Twist, queue_size=10)
	vel_msg = Twist()
	side_length = s
	rotations = r

	current_rotation = 0
	while current_rotation < rotations and not rospy.is_shutdown():
		move_in_line(side_length,vel_msg,pub)
		rotate(vel_msg,pub)
		current_rotation+=0.25

def move_square():
	rospy.init_node('move_square',anonymous = True)
	print("Escriba el lado del cuadrado y el numero de rotaciones: ")
	side_length = input()
	n_rotations = input()
	s = float(side_length)
	r = float(n_rotations)
	handle_move_square(s,r)
	rospy.spin()


def move_in_line(side_length,vel_msg,pub):
	vel_msg.linear.x = speed
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0

	# t0 = rospy.Time.now().to_sec()
	# distance_travelled = 0
 
	# while distance_travelled < side_length:
	# 	pub.publish(vel_msg)
	# 	t1 = rospy.Time.now().to_sec()
	# 	distance_travelled = speed*(t1-t0)
 
 	# t0 = rospy.Time.now().to_sec()
	# distance_travelled = 0
 
	pub.publish(vel_msg)
	sleep = side_length / speed
	time.sleep(sleep)
  
	vel_msg.linear.x = 0
	pub.publish(vel_msg)

	return

def rotate(vel_msg,pub):
	vel_msg.angular.z = speed*5
	# t0	= rospy.Time.now().to_sec()
	# angle_travelled = 0

	# while ( angle_travelled < PI/2.0 ):
	# 	pub.publish(vel_msg)
	# 	t1 = rospy.Time.now().to_sec()
	# 	angle_travelled = angular_speed*(t1-t0)

	# vel_msg.angular.z = 0
	# pub.publish(vel_msg)
   
 
	pub.publish(vel_msg)
	sleep = (PI/2.0) / vel_msg.angular.z
	time.sleep(sleep)
 
	vel_msg.angular.z = 0
	pub.publish(vel_msg)

if __name__ == '__main__':
	try:
		move_square()
	except rospy.ROSInterruptException:
		pass
