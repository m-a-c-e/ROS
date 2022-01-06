#!/usr/bin/env python       

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import time

# used to extract the information obtained by the subscriber
def callback(data, pub):
	# need to know the width of the image taken by rospycam
	x_img = data.x
	x_centre = data.z

	
	velocity = Twist()
	
	linear = Vector3()
	linear.x = 0
	linear.y = 0
	linear.z = 0
	
	angular = Vector3()
	angular.x = 0
	angular.y = 0
	
	velocity.linear = linear

#	print(abs(x_img - x_centre))

	pix_diff = x_img - x_centre

	if(abs(pix_diff) < 18):
		angular.z = 0.0	
	else:
		if(pix_diff < 0):
			angular.z = 0.15
		else:
			angular.z = -0.15
	velocity.angular = angular
	pub.publish(velocity)


def sub_and_pub():
	rospy.init_node('rotate_robot', anonymous=False)
	velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	rospy.Subscriber("/find_object/co_ordinates", Point, callback, velocity_pub, queue_size=1, buff_size=2**24)
	while not rospy.is_shutdown():
		time.sleep(1)
		
if __name__ == '__main__':
	try:
		sub_and_pub()
	except rospy.ROSInterruptException:
		pass
