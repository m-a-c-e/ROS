#!/usr/bin/env python       

import numpy as np
import rospy
import rospy
import time
import math

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from math import sqrt




class wallEOdometry:
	def __init__(self):
		self.odom_sub = 	rospy.Subscriber('/odom', Odometry, self.update_Odometry)	 				# handle to get the position
		self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 		 					# handle to publish velocity
		self.lidar_sub = 	rospy.Subscriber('/scan', LaserScan, self.detect_object, queue_size=1)		# for object detection
		
		self.Init = True						# only for when the robot publishes the first odometry data
		self.Init_pos = Vector3(0, 0, 0)				
		self.Init_ang = 0 						# radians
		self.globalPos = Vector3(0, 0, 0)
		self.globalAng = 0						# radians
		self.velocity = Twist()
		self.velocity.linear = Vector3(0, 0, 0)
		self.velocity.angular = Vector3(0, 0, 0)
		self.ao_counter = 0
		
		# changed here
		self.d_ob = float('inf')
		self.obstacle_dist = None
		self.closest_dist = None
		self.global_goal = {'g1': [1.5, 0]}
		self.ranges = []
				
		# define the state machine
		# Initial state = 'M'
		# State description:
		#  'MF':  Move forward state - Robot moves forward (need a controller to move in a straight line)
		#  'G1:  Robot within 5 cm of (x = 1.5 m, y = 0.0 m)
		#  'G2': Robot within 5 cm of (x = 1.5 m, y = 1.4 m)
		#  'G3': Robot within 5 cm of (x = 0 m, y = 1.4 m)
		#  'AO': Robot facing an obstacle within 15 cm of -20 deg, +20 deg in front (need to get the minimum)		
		self.state = 'MF'		# initial state

	def detect_object(self, data):
		ranges = data.ranges
		self.ranges = ranges
		min_dist = 100
		for i in range(0, 90):			 
			if(ranges[i] < 0.11):
				continue
			if(ranges[i] < min_dist):
				min_dist = ranges[i]
		for i in range(270, 360):		
			if(ranges[i] < 0.11):
				continue
			if(ranges[i] < min_dist):
				min_dist = ranges[i]
		self.closest_dist = min_dist
		self.d_ob = ranges[359]
	
	def update_Odometry(self, Odom):
		position = Odom.pose.pose.position
		#Orientation uses the quaternion aprametrization.
		#To get the angular position along the z-axis, the following equation is required.
		q = Odom.pose.pose.orientation
		orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
		if self.Init:
			#The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
			self.Init = False
			self.Init_ang = orientation			 				# radians
			self.globalAng = self.Init_ang						# radians
			Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])
			self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y				# meters
			self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y				# meters
			self.Init_pos.z = position.z		
		Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
		#We subtract the initial values
		self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x		# meters
		self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y		# meters
		self.globalAng = orientation - self.Init_ang			# radians
		self.velocity_pub.publish(self.velocity)
		
	def set_state(self):			
		# Set to AO -> condition: when obstacle 'in front'
		if( abs(self.d_ob - 0.30) < 0.05 and self.state is not 'AO' ):
			FSM.velocity.linear.x = 0
			FSM.velocity.angular.x = 0			# changed here
			time.sleep(1)
			self.state = 'AO'
			#Closest distance to obstace gets set only when system enters AO and remains fixed for this state
			self.obstacle_dist = self.closest_dist
			print("obstacle_dist", self.obstacle_dist)
		
		# Set to G1 -> condition: when reaches G1
		if( abs(self.globalPos.x - 1.5) < 0.01 and abs(self.globalPos.y - 0) < 0.05 and list(self.global_goal.keys())[0] is 'g1' ):
			self.state = 'G1'
		
		# Set to G2 -> condition: when reaches G2
		if( abs(self.globalPos.x - 1.5) < 0.05 and abs(self.globalPos.y - 1.4) < 0.07 and list(self.global_goal.keys())[0] is 'g2'):
			self.state = 'G2'
		
		# Set to G3 -> condition: when reaches G3
		if( abs(self.globalPos.x - 0) < 0.15 and abs(self.globalPos.y - 1.4) < 0.15 and list(self.global_goal.keys())[0] is 'g3' ):
			self.state = 'G3'
#		if( FSM.dist_to_goal() < 0.2 and list(self.global_goal.keys())[0] is 'g3' ):
#			self.state = 'G3'
#### Obstacle clearing functions ####
	def rotate_to_ao(self, angle):
		print("Entered rotate_to_ao!!!\n")
		print("angle (constant) = ", angle)
		
		
		
		init_angle = FSM.globalAng
		
		print("initial angle (constant) = ", init_angle)
		
		'''
		if( init_angle > 0 and angle < 0 ):
			FSM.velocity.angular.z = -0.2
			while(True):
				if( abs(FSM.globalAng - init_angle) > abs(angle) ):
					FSM.velocity.angular.z = 0
					return
				 
		if( init_angle < 0 and angle > 0 ):
			FSM.velocity.angular.z = 0.2
			while(True):
				if( abs(FSM.globalAng - init_angle) > abs(angle) ):
					FSM.velocity.angular.z = 0
					return

		if( init_angle > 0 and angle > 0 ):
			FSM.velocity.angular.z = 0.2
			while(True):
				if(FSM.globalAng < 0):
					dummy_ang = 2 * 3.1415 - init_angle + FSM.globalAng
					if( abs(dummy_ang) > abs(angle)):
						FSM.velocity.angular.z = 0
						return
				else:
					if(abs(FSM.globalAng - init_angle) > abs(angle) ):
						FSM.velocity.angular.z = 0
						return
		
		if( init_angle < 0 and angle < 0 ):
			FSM.velocity.angular.z = -0.2
			while(True):
				if(FSM.globalAng > 0):
					dummy_ang = 2 * 3.1415 - FSM.globalAng + init_angle
					if( abs(dummy_ang) > abs(angle) ):	
						FSM.velocity.angular.z = 0
						return
				else:
					if( abs(dummy_ang - init_angle) > abs(angle) ):
						FSM.velocity.angular.z = 0
						return
		'''

		final_angle = init_angle + angle
		if (init_angle>0) and (angle>0) and (final_angle>3.14):
			final_angle = final_angle - (2*3.14)
		elif (init_angle<0) and (angle<0) and (final_angle<3.14):
                        final_angle = - ( (2*3.14)+final_angle)

		print("final_angle (constant) = ", final_angle)
		
		while (abs(FSM.globalAng - final_angle) > 0.1):
			
			if (angle > 0):
				FSM.velocity.angular.z = 0.2
			else:
				FSM.velocity.angular.z = -0.2		
		FSM.velocity.angular.z = 0
		
		print("finished turning in ao")
		print("FSM.globalAng = ", FSM.globalAng)
		print(" final_angle - FSM.globalAng = ", FSM.globalAng - final_angle)
		
		return
			
	def rotate_to_goal(self):
		print("Entered rotate to goal!!")
		
		
		
		goal = list(self.global_goal)[0]
		goal_x = self.global_goal[goal][0]
		goal_y = self.global_goal[goal][1]
		curr_x = self.globalPos.x
		curr_y = self.globalPos.y
		y = goal_y - curr_y
		x = goal_x - curr_x
		
		angle_g = math.atan2(y, x)
		angle_robot = self.globalAng
		z = 0
		
		
		print("angle_g before = ", angle_g)
	#	if (list(self.global_goal.keys())[0] is 'g3' and self.ao_counter == 2):
	#			if angle_g >0:
	
	#				angle_g = -(3.14-angle_g)
	#			else:
		
	#				angle_g = 3.14+angle_g
		
		print("angle_g after = ", angle_g)

		if(angle_robot > 0 and angle_g > 0):
				if(angle_g > angle_robot):
					z = 0.2
				else:
					z = -0.2
		
		if(angle_robot < 0 and angle_g < 0):
			if(angle_g > angle_robot):
				z = -0.2
			else:
				z = 0.2
				
		if(angle_g > 0 and angle_robot < 0):
			if( angle_g + abs(angle_robot) < 3.14 ):
				z = 0.2
			else:
				z = -0.2
				
		if(angle_g < 0 and angle_robot > 0):
			if( abs(angle_g) + angle_robot < 3.14 ):
				z = -0.2
			else:
				z = 0.2
		i = 0
		print("initial_angle = ", FSM.globalAng)
		FSM.velocity.angular.z = z
		while(True):
			mod_globalAng  = FSM.globalAng
			i = i + 1
			
			# only for g3
			if(list(self.global_goal.keys())[0]):
			#		print("FSM.globalAng = ", FSM.globalAng)
					i = 0
					if (FSM.globalAng > 3.14):
						mod_globalAng  = FSM.globalAng - 2*3.14
					else:
						mod_globalAng  = FSM.globalAng
			
			# for rest
			if(abs(mod_globalAng - angle_g) < 0.25):				
				FSM.velocity.angular.z = 0
				break
				
		print("done rotating in rotate_to_goal")
		print("FSM.globalAng = ", FSM.globalAng)
		print("FSM.globalAng - angle_g =", FSM.globalAng - angle_g)
		
	def rotate_to(self, init_angle, angle):

		if(angle > 0):
                	FSM.velocity.angular.z = 0.15
                        while( ( (init_angle * FSM.globalAng) > 0 and abs(init_angle - FSM.globalAng) < angle ) or 
                                   ( (init_angle * FSM.globalAng) < 0 and (2 * 3.14159 - abs(init_angle) + abs(FSM.globalAng)) < angle) ):
                                i = 0
                                # # print("init_angle = ", init_angle * 180 / 3.141)
                                # # print("FSM globalAng = ", FSM.globalAng * 180 / 3.141)
                                # # print("angle = ", angle)

                else:
                        FSM.velocity.angular.z = -0.15
                        while( ( (init_angle * FSM.globalAng) > 0 and abs(init_angle - FSM.globalAng) < abs(angle) ) or 
                                   ( (init_angle * FSM.globalAng) < 0 and (2 * 3.14159 - abs(init_angle) + abs(FSM.globalAng)) < abs(angle) ) ):
                                i = 0
                                # # print("init_angle = ", init_angle * 180 / 3.141)
                                # # print("FSM globalAng = ", FSM.globalAng * 180 / 3.141)
                                # # print("angle = ", angle)



	#	if(angle > 0):
	#		FSM.velocity.angular.z = 0.25
	#		while(abs(init_angle - FSM.globalAng) < angle):
	#			i = 0
	#	else:
	#		FSM.velocity.angular.z = -0.25
	#		while(abs(init_angle - FSM.globalAng) < abs(angle)):
	#			i = 0
	#	FSM.velocity.angular.z = 0

	def check_angle(self, angle):
		if(FSM.ranges[angle] < 0.11 or FSM.ranges[angle] > 1):
			return True
		else:
			return False
			
###### Orientation ####
	def angle_to_goal(self):
		goal = list(self.global_goal)[0]
		goal_x = self.global_goal[goal][0]
		goal_y = self.global_goal[goal][1]
		curr_x = self.globalPos.x
		curr_y = self.globalPos.y
		y = goal_y - curr_y
		x = goal_x - curr_x
		return_angle = math.atan2(y, x)		# returns between -pi and +pi radians
		return return_angle
	
	def dist_to_goal(self):
		goal = list(self.global_goal)[0]
		goal_x = self.global_goal[goal][0]
		goal_y = self.global_goal[goal][1]
		curr_x = self.globalPos.x
		curr_y = self.globalPos.y
		y = goal_y - curr_y
		x = goal_x - curr_x
		dist_value = math.sqrt(x * x + y * y)
		return dist_value					# returns in meters
		
if __name__ == '__main__':
	rospy.init_node('move_fwd', anonymous=False)

	# create the state machine with initial state as 'MF'
	FSM = wallEOdometry()
	while not rospy.is_shutdown():
		try:
			previous_state = FSM.state
			FSM.set_state()
			if(previous_state is not FSM.state):
				print("state changed from ", previous_state, "to ", FSM.state)
			curr_state = FSM.state
		
			if(curr_state is 'AO'):
				
				FSM.ao_counter += 1
				print('/n')
				print("avoiding obstacle")					
				
				obstacle_dist = FSM.obstacle_dist
				FSM.velocity.linear.x = 0
				FSM.velocity.angular.z = 0
				
				######################
				hypo = FSM.d_ob
				perpen = FSM.obstacle_dist
				a_cos_value = perpen / hypo		
				angle_to_rotate = 1.5708 - math.acos(perpen/hypo)
				######################
				
				#checks how the object is placed
				if (FSM.ranges[5] >= FSM.ranges[355]):
					#object is at a negative slope/ perpendicular to path
					# and robot should rotate counter-clockwise/ positively
					r = 1.0
					
				else:
					#object is at a positive slope and robot should rotate clockwise/ negatively
					r = -1.0
					angle_to_rotate = -1.0 * angle_to_rotate
					
				#print("globalAng = ", FSM.globalAng * 180 / 3.141)
				#print("angle_to_rotate = ", angle_to_rotate * 180 / 3.141)
				
				FSM.rotate_to_ao(angle_to_rotate)
			#	FSM.rotate_to(FSM.globalAng, angle_to_rotate)
				#robot moves linearly till the obstacle is cleared and then turns 90 degree left/right
				obstacle_cleared = False
				if(r == 1):
					angles = [315, 270, 225]
				else:
					angles = [45, 90, 135]
				counter = 0
				while not obstacle_cleared:
					FSM.velocity.linear.x = 0.20
					if(FSM.check_angle(angles[counter])):
						counter += 1
					if(counter == 3):
						obstacle_cleared = True
						FSM.velocity.linear.x = 0
						if(r == 1):
							print("distances:")
							print("270: ", FSM.ranges[270])
							print("225: ", FSM.ranges[225])
							print("315: ", FSM.ranges[315])
						else:
							print("distances:")
							print("90: ", FSM.ranges[270])
							print("45: ", FSM.ranges[225])
							print("135: ", FSM.ranges[315])
							
				FSM.rotate_to_goal()
				print("Inisde AO rotated towards", list(FSM.global_goal.keys()))
				FSM.state = 'MF'
				print('/n')
				
			if(curr_state is 'MF'):
				# TO DO:
				# 1. Set desired velocity
				# 2. Implement controller to move in a straight line with minimal error		
				FSM.velocity.linear.x = 0.20
				kp_ang = 0.5
				angular_error = FSM.angle_to_goal() - FSM.globalAng

					
				if(list(FSM.global_goal.keys())[0] is 'g3'):
					if (FSM.globalAng > 3.14):
						mod_globalAng  = FSM.globalAng - 2*3.14
					else:
						mod_globalAng  = FSM.globalAng
						
					angular_error = FSM.angle_to_goal() - mod_globalAng
					
					if(kp_ang * angular_error > 0.1):
						FSM.velocity.angular.z = 0.1
					elif(kp_ang * angular_error < -0.1):
						FSM.velocity.angular.z = -0.1
					else:			
						# Proportional control
						FSM.velocity.angular.z = kp_ang * angular_error
			
					
					
					
				if( list(FSM.global_goal.keys())[0] is not 'g3'):
					if(kp_ang * angular_error > 0.1):
						FSM.velocity.angular.z = 0.1
					elif(kp_ang * angular_error < -0.1):
						FSM.velocity.angular.z = -0.1
					else:			
						# Proportional control
						FSM.velocity.angular.z = kp_ang * angular_error
				
#				i = 0
#				if(list(FSM.global_goal.keys())[0] is not 'g3' and i < 100):
#					print("angular velocity = ", FSM.velocity.angular.z)
#					print("ANGLE TO GOAL = ", FSM.angle_to_goal(), " and GLOBAL ANG = ", FSM.globalAng)
#					i = i + 1

				
#				if(list(FSM.global_goal.keys())[0] is 'g3' and i < 10):
#					print("x = ", FSM.globalPos.x, " y = ", FSM.globalPos.y)
#					i = i + 1
			if(curr_state is 'G1'):
				# TO DO
				print('/n')
				
				print("Reached G1")
				
				FSM.velocity.linear.x = 0
				FSM.velocity.angular.z = 0
				
				FSM.global_goal = {'g2': [1.5, 1.4]}		# set the location of new goal
				
				time.sleep(10)


				FSM.rotate_to_goal()

				print("rotated towards G2")

				FSM.state = 'MF'
				
				print('/n')

			if(curr_state is 'G2'):
				# TO DO
				print('/n')

				print("Reached G2")
				
				FSM.velocity.linear.x = 0
				FSM.velocity.angular.z = 0	
					
				FSM.global_goal = {'g3':[0, 1.4]}			# set the goal location
				
				time.sleep(10)
				
				FSM.rotate_to_goal()
				
				print("rotated towards G3")
				
				#FSM.Init = True
				
				#FSM.global_goal = {'g3':[1.5, 0]}
				
				FSM.state = 'MF'
				
				print('/n')
				
			if(curr_state is 'G3'):
				# TO DO
				print('/n')
				
				print("Reached G3")
				
				FSM.velocity.linear.x = 0
				FSM.velocity.angular.z = 0

				try:
					# loop forever until exit
					while(True):
						i = 0
				except rospy.ROSInterruptException:
					print("defd")
		except rospy.ROSInterruptException:
			print("defd")
