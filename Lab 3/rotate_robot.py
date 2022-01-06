#!/usr/bin/env python       

from sensor_msgs.msg import LaserScan
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import time

prev_angular_error = None
error_sum = 0
dis_forward = None

prev_linear_error = None
error_linear_sum = 0

kp = 0.01
kd = 0.001
ki = 0.0001

kp_linear = 0.003
kd_linear = 0.0001
ki_linear = 0.0009


# used to extract the information obtained by the subscriber
def callback(data, pub):
	# need to know the width of the image taken by rospycam

	x_img = data.x

	x_img = 0.808 * x_img - 64.2		# degrees (-31.1 to +31.1)

#	x_centre = data.z
	x_centre = 0.0				# degrees (0 deg)

	global prev_angular_error
	global prev_linear_error

	global kp
	global kd
	global ki

	global kp_linear
	global ki_linear
	global kd_linear

	global dis_forward
	global error_sum
	global error_linear_sum
	
	velocity = Twist()
	
	linear = Vector3()
	linear.x = 0
	linear.y = 0
	linear.z = 0
	
	angular = Vector3()
	angular.x = 0
	angular.y = 0
	
	velocity.linear = linear

##	print(abs(x_img - x_centre))

##	pix_diff is current error
	curr_angular_error = x_img - x_centre
	curr_linear_error = dis_forward - 30

#	print("x_img = ", x_img, " x_centre = ", x_centre)

	print("dis_forward = ", dis_forward)
	print("curr_linear_error = ", curr_linear_error)

	if(prev_angular_error == None):
		error_change = 0
	else: 
		error_change = curr_angular_error - prev_angular_error

	if(prev_linear_error == None):
		error_linear_change = 0
	else:
		error_linear_change = curr_linear_error - prev_linear_error

	prev_angular_error = curr_angular_error
	prev_linear_error = curr_linear_error

	error_sum = error_sum + curr_angular_error
	error_linear_sum = error_linear_sum + curr_linear_error
	print("error_linear_sum = ", error_linear_sum)
	print("error_sum = ", error_sum)

	angular.z = kp * curr_angular_error + kd * error_change + ki * error_sum
	angular.z = -1 * angular.z
	

	if(curr_linear_error < 0):
		linear.x = 1.2 * kp_linear * curr_linear_error
	else:
		linear.x = kp_linear * curr_linear_error + kd_linear *  error_linear_change + ki_linear * error_linear_sum
	linear.x = linear.x
	print("linearx = ", linear.x)


	if(curr_linear_error > 80):
		angular_threshold = 7
	else:
		angular_threshold = 3

	print("angular threshold ", angular_threshold)	
	if(abs(curr_angular_error) < angular_threshold):
		angular.z = 0.0	
		error_sum = 0
		## object found
		if(data.z == -1000 or (data.z == 1000 and abs(curr_linear_error) < 2)):
			linear.x = 0
			error_linear_sum = 0
	# rotating towards the diamond
	else:
		linear.x = 0.0
		error_linear_sum = 0

	## make the robot move towards the object based on lidar and image data
	velocity.linear = linear
	velocity.angular = angular
	pub.publish(velocity)


def callback_lidar(data):
	global dis_forward
	ranges = data.ranges
	dis_forward = 100 * ranges[359]
	print("dis_forward = ", dis_forward)

def sub_and_pub():
	rospy.init_node('rotate_robot', anonymous=False)

	velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	rospy.Subscriber("/find_object/co_ordinates", Point, callback, velocity_pub, queue_size=1, buff_size=2**24)

   	rospy.Subscriber('/scan', LaserScan, callback_lidar, queue_size=1)

	while not rospy.is_shutdown():

		time.sleep(1)
		
if __name__ == '__main__':
	try:
		sub_and_pub()
	except rospy.ROSInterruptException:
		pass
