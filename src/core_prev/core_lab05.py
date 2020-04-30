#!/usr/bin/env python

import numpy as np
import math
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from autonomy.msg import motors, lines, distance, servos #, leds

# import messages
from fiducial_msgs.msg import *
from geometry_msgs.msg import *

class autonomy(object):

	def __init__(self):
		##USER PARAMETERS
		self.integral = 0
		self.setpoint = 0.25
		self.arucoX = 0
		self.arucoY = 0
		self.arucoZ = 0
		self.found = False
		self.previous_error = 0
		self.previous_error_angle = 0
		self.lateral_error_thres = 0.1
		##Initiate variables
		self.leftLine = 0
		self.midLine = 0
		self.rightLine = 0
		self.distance = 0
		self.leftSpeed = 0
		self.rightSpeed = 0
		self.pan = 0
		self.tilt = 0
		self.bridge = CvBridge()

		#Setup Publishers
		self.motorPub = rospy.Publisher('motors', motors, queue_size=10)
		self.servoPub = rospy.Publisher('servos', servos, queue_size=10)

		# camera calibration
		self.blobpub = rospy.Publisher('imageProc',Image, queue_size=10)
		#self.LEDpub = rospy.Publisher('leds', leds, queue_size=10)


		#Create Subscriber callbacks
		def lineCallback(data):
			self.leftLine = data.leftLine
			self.midLine = data.midLine
			self.rightLine = data.rightLine

		def distanceCallback(data):
			self.distance = data.distance


		def imageProcessing(data):
			try:
				frame=self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
			except CvBridgeError as e:
				print(e)

			##Place image processing code here!
			print("I came here")
			# Setup SimpleBlobDetector parameters.
			params = cv2.SimpleBlobDetector_Params() 
			# Filter by Area.
			params.filterByArea = True
			params.minArea = 1500

			# Filter by Circularity
			params.filterByCircularity = True
			params.minCircularity = 0
			 
			# Filter by Convexity
			params.filterByConvexity = True
			params.minConvexity = 0.5

			# Set up the detector with default parameters.
			detector = cv2.SimpleBlobDetector_create(params)

			# Detect blobs.
			keypoints = detector.detect(frame)
			 
			# Draw detected blobs as red circles.
			# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
			im_with_keypoints = cv2.drawKeypoints(frame, keypoints, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
			self.blobpub.publish(self.bridge.cv2_to_imgmsg(im_with_keypoints,"bgr8"))	

		# callback for Fiducial_msgs
		def fiducialNav(data):
			# imageTime = data.header.stamp
			# print (imageTime - rospy.Time.now())/-1e6
			# print "*****"

			# For id = 5
			for m in data.transforms:
				print(m.fiducial_id)
				if m.fiducial_id == 5:
					trans = m.transform.translation
					self.arucoX = trans.x
					self.arucoY = trans.y
					self.arucoZ = trans.z
					self.found = True
				else:
					self.found = False


		#Subscribe to topics
		# rospy.Subscriber('raspicam_node/image_rect_color',Image,imageProcessing)
		rospy.Subscriber('lines', lines, lineCallback)
		# rospy.Subscriber('distance', distance, distanceCallback)
		rospy.Subscriber('/fiducial_transforms',FiducialTransformArray,fiducialNav)

		# initialize
		rospy.init_node('core', anonymous=True)
		self.rate = rospy.Rate(10)

	def publishMotors(self):
		motorMsg = motors()
		motorMsg.leftSpeed = self.leftSpeed
		motorMsg.rightSpeed = self.rightSpeed
		# rospy.loginfo(motorMsg)
		self.motorPub.publish(motorMsg)

	def publishServo(self):
		servoMsg = servos()
		servoMsg.pan = self.pan
		servoMsg.tilt = self.tilt
		# rospy.loginfo(servoMsg)
		self.servoPub.publish(servoMsg)


	def runner(self):
		# self.start = time.time()

		while not rospy.is_shutdown():
			

			self.move()

			##Leave these lines at the end
			# self.publishMotors()
			# self.publishServo()
			self.rate.sleep()


	def move(self):

		max_speed = 0.2
		min_speed = 0.05
		kp_f = 0.8
		kd_f = 40

		# A fiducial was detected since last iteration of this loop
		if self.found:

			forward_error = self.arucoZ - 0.8
			lateral_error = self.arucoX

			if abs(lateral_error) > self.lateral_error_thres :
				self.stopCar()
				self.wait(0.5)
				self.fix_angle()
				
			# Set the forward speed based distance
			linSpeed = kp_f * forward_error + kd_f * (forward_error - self.previous_error)/10
			if linSpeed <= -max_speed:
				self.forward(-max_speed)
			elif linSpeed > -max_speed and linSpeed < -min_speed:
				self.forward(linSpeed)
			elif linSpeed >= max_speed:
				self.forward(max_speed)
			elif linSpeed > min_speed and linSpeed < max_speed:
				self.forward(linSpeed)
			else:
				self.stopCar()

			print "Errors: forward %f lateral %f speed %f" % \
			  (forward_error, lateral_error, linSpeed)
			self.previous_error = forward_error

		else:
			self.stopCar()

		self.rate.sleep()


	def fix_angle(self):

		forward_error = self.arucoZ - 0.8
		lateral_error = self.arucoX

		while abs(lateral_error) > self.lateral_error_thres :
			print "Errors: forward %f lateral %f " % (forward_error, lateral_error)
			if lateral_error > self.lateral_error_thres :
				self.turnRight(0.2)
			elif lateral_error < -self.lateral_error_thres :
				self.turnLeft(0.2)
			else:
				print("Done Alignment................................")
				self.stopCar()
				self.wait(1)
				exit

			self.rate.sleep()
			lateral_error = self.arucoX

	def forward(self, speed):
		self.leftSpeed = speed
		self.rightSpeed = speed		
		self.publishMotors()

	def turnLeft(self, speed):
		self.leftSpeed = -speed
		self.rightSpeed = speed
		self.publishMotors()

	def turnRight(self, speed):
		self.leftSpeed = speed
		self.rightSpeed = -speed
		self.publishMotors()

	def stopCar(self):
		self.leftSpeed = 0
		self.rightSpeed = 0
		self.publishMotors()

	def wait(self,dur):
		start = time.time()
		while True:
			if time.time() - start > dur:
				break	
		self.stopCar()


if __name__ == '__main__':
	autonomy().runner()

# hello