#!/usr/bin/env python       
import numpy as np
import rospy
import rospy
import time
import math
import cv2
import shutil

import sys
import csv
import time
import os

import shutil

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage

from math import sqrt

class wallEOdometry:
	def __init__(self):
		self.odom_sub = 	rospy.Subscriber('/odom', Odometry, self.update_Odometry)	 				# handle to get the position
		self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 		 					# handle to publish velocity
		self.lidar_sub = 	rospy.Subscriber('/scan', LaserScan, self.detect_object, queue_size=1)		# for object detection
		self.cam_sub = 		rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, self.get_image, queue_size=1)

		self.Init = True						# only for when the robot publishes the first odometry data
		self.Init_pos = Vector3(0, 0, 0)				
		self.Init_ang = 0 						# radians
		self.globalPos = Vector3(0, 0, 0)
		self.globalAng = 0						# radians
		self.velocity = Twist()
		self.velocity.linear = Vector3(0, 0, 0)
		self.velocity.angular = Vector3(0, 0, 0)
		self.ao_counter = 0
		self.raw_angle = 0
		self.kp_ang = 0.05
		self.image = None
		
		# changed here
		self.d_ob = float('inf')
		self.ranges = []
		# define the state machine
		self.state = 'MF'		# initial state

	def get_image(self, data):
		img = data.data
		np_arr = np.fromstring(img, np.uint8)
		image_np = cv2.imdecode(np_arr, 1)
		self.image = image_np

	def detect_object(self, data):
		self.ranges = data.ranges
		self.d_ob = self.ranges[359]
		#print(self.d_ob)

	def update_Odometry(self, Odom):
		position = Odom.pose.pose.position
		#Orientation uses the quaternion aprametrization.
		#To get the angular position along the z-axis, the following equation is required.
		q = Odom.pose.pose.orientation
		orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)) * 180 / 3.1415
		if orientation < 0:
			orientation = 360 + orientation
		self.raw_angle = orientation
		self.velocity_pub.publish(self.velocity)

	def set_state(self):					
        # TO DO:
        
		''' MF '''
		if(self.d_ob - 0.50 < 0.05 and self.state is 'MF'):
			self.velocity.linear = Vector3(0, 0, 0)
			self.velocity.angular = Vector3(0, 0, 0)
			time.sleep(1)
			self.state = 'FW'
			
			
##############################################################################
###### FINAL PROJECT FUNCTIONS ###############################################
##############################################################################
	def rotate_by(self, amount, sign):
        # makes the robot rotate by 'amount' degrees in 'sign' direction. 1: anti-clockwise, -1: clockwise
        # amount and sign must be same
        
		initial_angle = self.raw_angle
		threshold = 2   						# degrees
		final_angle = initial_angle + amount

        # angle wrap
		if final_angle >= 360:
			final_angle = final_angle - 360

		if final_angle < 0:
			final_angle = final_angle + 360

		# print initial angle and final angle
		print("amount = ", amount, "direction = ", sign)

		print("initial angle = ", initial_angle, " final angle = ", final_angle)

        # start to rotate
		self.velocity.angular = Vector3(0, 0, 0.1 * sign)
    
		# break condition
		while(True):
			if abs(self.raw_angle - final_angle) <= threshold:
				break 
        # stop rotating
		self.velocity.angular = Vector3(0, 0, 0)

        # Done
		print("Done! initial_angle = {}, final_angle = {}".format(initial_angle, final_angle))
        
    
	def calc_goal_ang(self, sign_detected):
		# Outputs the angle by which to rotate and direction depending on the sign

		print("Sign detected = ", sign_detected)

		goal_ang = 0
		direction = 0

		''' Left '''
		if sign_detected == 1:
			goal_ang = 90
			direction = 1
			
		''' Right '''
		if sign_detected == 2:
			goal_ang = -90
			direction = -1

		''' U - Turn '''
		if sign_detected == 3 or sign_detected == 4:
			#Robot rotates anti-clockwise by 180 degrees
			goal_ang = 180
			direction = 1
		
		if sign_detected == 5:
			goal_ang = 0
			direction = 1

		return (goal_ang, direction)
		
		
	def detect_sign(self, filepath):
		imageDirectory = './detect_sign_train/'

		with open(imageDirectory + 'train.txt', 'r') as f:
			reader = csv.reader(f)
			lines = list(reader)

		# this line reads in all images listed in the file in GRAYSCALE, and resizes them to 33x25 pixels
		train = np.array([np.array(cv2.resize(cv2.imread(imageDirectory +lines[i][0]+".jpg",1),(33,25))) for i in range(len(lines))])

		# here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants)
		train_data = train.flatten().reshape(len(lines), 33*25*3)
		train_data = train_data.astype(np.float32)
		#print(train_data)

		# read in training labels
		train_labels = np.array([np.int32(lines[i][1]) for i in range(len(lines))])

		### Train classifier
		knn = cv2.ml.KNearest_create()
		knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

		k = 2

		#original_img = cv2.imread(filepath,1)
		original_img = filepath
		image = original_img[30:original_img.shape[0]-70, 0:original_img.shape[1]-50]
		grey= cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		edges= cv2.Canny(grey, 50,200)

		contour, hier= cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
		sorted_contours= sorted(contour, key=cv2.contourArea, reverse= True)
		sorted_contours = sorted_contours[0:1]

		for (i,c) in enumerate(sorted_contours):
			x,y,w,h= cv2.boundingRect(c)
			cropped_contour= original_img[y:y+h, x:x+w]
		if len(sorted_contours) ==0:
			   cropped_contour= image
		test_img = np.array(cv2.resize(cropped_contour,(33,25)))
		test_img = test_img.flatten().reshape(1, 33*25*3)
		test_img = test_img.astype(np.float32)

		ret, results, neighbours, dist = knn.findNearest(test_img, k)
		return(ret)

		
##############################################################################
######### MAIN ###############################################################
#############################################################################

if __name__ == '__main__':
	rospy.init_node('move_fwd', anonymous=False)
	
	# create the state machine with initial state as 'MF'
	FSM = wallEOdometry()
	time.sleep(2)
	initial_angle = FSM.raw_angle			   # update this when changing states
	FSM.velocity.linear = Vector3(0.1, 0, 0)
	count =  0
	signCounter = 0
	
	while not rospy.is_shutdown():
         
		try:
			previous_state = FSM.state
			FSM.set_state()

			if previous_state is not FSM.state:
				print(previous_state, " --> ", FSM.state)

            ##### MOVE FORWARD STATE ##########################################
			if FSM.state == 'MF':
				
				FSM.velocity.linear = Vector3(0.1, 0, 0)
				min_dist = 100
				min_dist_ang = -1
				for i in range(0, 91):
					if(FSM.ranges[i] < 0.11):
						continue
					if(FSM.ranges[i] < min_dist):
						min_dist = FSM.ranges[i]
						min_dist_ang = i
				for i in range(270, 360): 
					if(FSM.ranges[i] < 0.11):
						continue
					if(FSM.ranges[i] < min_dist):
						min_dist = FSM.ranges[i]
						min_dist_ang = i
				
				# move away from wall
				if min_dist <= 0.2:
					print("entered")
					FSM.velocity.linear = Vector3(0, 0, 0)
					FSM.velocity.angular = Vector3(0, 0, 0)
					time.sleep(2)
					if i >= 0 and i <= 90:
						# rotate right
						print("entered rotate right")
						FSM.rotate_by(-3, -1)
						initial_angle = initial_angle - 1
						if initial_angle < 0:
							initial_angle = initial_angle + 360
					else:
						# rotate left
						print("entered rotate left")
						FSM.rotate_by(3, 1)
						initial_angle = initial_angle + 1
						if initial_angle >= 360:
							initial_angle = initial_angle - 360
				
				count = count + 1
				angular_error = 0
				# implement angular controller
				if (initial_angle >= 0 and initial_angle <= 90) and (FSM.raw_angle >= 270):
					# initial angle is in 1st quad and raw_angle is in 4th quad
					angular_error = initial_angle + (360 - FSM.raw_angle)
				elif (FSM.raw_angle >= 0 and FSM.raw_angle <= 90) and (initial_angle >= 270):
					# raw_angle in 1st quad and initial_angle is in 4th quad
					angular_error = -1 * (FSM.raw_angle + (360 - initial_angle))
				else:
					angular_error = initial_angle - FSM.raw_angle
					
				FSM.velocity.linear = Vector3(0.1, 0, 0)
				FSM.velocity.angular = Vector3(0, 0, FSM.kp_ang * angular_error)

            ##### FACE THE WALL ###############################################
			if FSM.state == 'FW':
				FSM.velocity.linear = Vector3(0, 0, 0)
				FSM.velocity.angular = Vector3(0, 0, 0)
				time.sleep(1)
				min_dist = 100
				min_dist_ang = -1
				for i in range(0, 21):
					if(FSM.ranges[i] < 0.11):
						continue
					if(FSM.ranges[i] < min_dist):
						min_dist = FSM.ranges[i]
						min_dist_ang = i
				for i in range(339, 360): 
					if(FSM.ranges[i] < 0.11):
						continue
					if(FSM.ranges[i] < min_dist):
						min_dist = FSM.ranges[i]
						min_dist_ang = i

				# Face the wall
				direction = 0
				if min_dist_ang > 20:
					direction = -1
					min_dist_ang = -1 * (360 - min_dist_ang)
				else:
					direction = 1
					
				# rotated to face the wall
				FSM.rotate_by(min_dist_ang, direction)

				# detect sign
				sign = FSM.detect_sign(FSM.image)
				if sign == 0:
					signCounter = signCounter + 1
					
				if signCounter == 10:
					signCounter = 0
					sign = 5
				
				if sign != 0:				
					# calculate angle based on sign
					amount, direction = FSM.calc_goal_ang(sign)
	
	                # Rotate based on sign
					FSM.rotate_by(amount, direction)
					
				if sign == 5:
					FSM.state = 'END'
				else:
					initial_angle = FSM.raw_angle
					FSM.state = 'MF'
					print("FW to MF\n")
				
			
			''' END '''
			if FSM.state == 'END':
				FSM.velocity.linear = Vector3(0, 0, 0)
				FSM.velocity.angular = Vector3(0, 0, 0)
                 
		except rospy.ROSInterruptException:
			print("Error")
