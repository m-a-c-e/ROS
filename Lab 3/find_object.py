#!/usr/bin/env python       

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
import diamond_finder
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import time

# import the find_image file

def callback(data, pub):		# will be called as fast as the refresh rate defined below

    img = data.data				# get the image data (uint8 [])
#    pub = args[0]				# will be used to publish the co-ordinates
    np_arr = np.fromstring(img, np.uint8)
    image_np = cv.imdecode(np_arr, 1)

    coords = diamond_finder.finding_template('template.jpg', image_np)
    print(coords)

#    coords = [[0, 0]]

    p = Point()
    p.z = (image_np.shape[0])/2

    #Choose the coordinates that need to be considered, here is just taken as the first diamond found
    if (coords != []):
	print(coords)
        co_ord = coords[0]
	p.x = co_ord[0]
	p.y = co_ord[1]
	p.z = 1000
	
	print("x = ", p.x)
	print("y = ", p.y)
	print("z = ", p.z)
	# once the image is processed
    else:
	p.x = p.z
	p.z = -1000 

    pub.publish(p)



def sub_and_pub():
	# 1. Initialise node
	rospy.init_node('find_object', anonymous=False)

	# 2. Initialise the subscriber and publisher handle
	img_pub = rospy.Publisher('/find_object/co_ordinates', Point, queue_size=1)	# will be used to publish the co-ordinates
	
	rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, callback, img_pub, queue_size=1)		# use callback to process the image data


	# 4. Loop at above rate, getting information and sending information
	while not rospy.is_shutdown():
#		rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, callback, img_pub, queue_size=1)		# use callback to process the image data
		#rate.sleep()

		time.sleep(1)
		
if __name__ == '__main__':
	try:
		sub_and_pub()
	except rospy.ROSInterruptException:
		pass







