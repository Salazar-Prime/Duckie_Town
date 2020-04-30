#!/usr/bin/env python

import numpy as np
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from autonomy.msg import motors, lines, distance, servos #, leds


class autonomy(object):

	def __init__(self):
		##USER PARAMETERS
		self.integral = 0
		self.setpoint = 0.25
		self.start = time.time()
		self.flag = 0

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
			#cv2.imwrite('test.jpg',frame)


		#Subscribe to topics
		rospy.Subscriber('raspicam_node/image',Image,imageProcessing)
		rospy.Subscriber('lines', lines, lineCallback)
		rospy.Subscriber('distance', distance, distanceCallback)

		rospy.init_node('core', anonymous=True)
		self.rate = rospy.Rate(100)

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

#	def publishLED(self):
#		LEDmsg = leds()
#		LEDmsg.r1 = 255
#		LEDmsg.g1 = 0
#		LEDmsg.b1 = 0
#		LEDmsg.r2 = 0
#		LEDmsg.g2 = 255
#		LEDmsg.b2 = 0
#		LEDmsg.r3 = 0
#		LEDmsg.g3 = 0
#		LEDmsg.b3 = 255
#		rospy.loginfo(LEDmsg)
#		self.LEDpub.publish(LEDmsg)

	def runner(self):
		# self.start = time.time()

		while not rospy.is_shutdown():
			
			if self.flag == 0:
				self.wait(5)
				self.flag = 1

			## Place code here
			if self.distance < 0.3:
				self.stopCar()
				self.scan()

			self.forward()

			
			##Leave these lines at the end
			# self.publishMotors()
			self.publishServo()
#			self.publishLED()
			self.rate.sleep()

	def forward(self):
		self.leftSpeed = 0.3
		self.rightSpeed = 0.3		
		self.publishMotors()

	def turnLeft(self):
		self.leftSpeed = -0.7
		self.rightSpeed = 0.7
		self.publishMotors()

	def turnRight(self):
		self.leftSpeed = 0.7
		self.rightSpeed = -0.7
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
	
	# averages distance from 50 scans
	def measureDist(self, numScans):
		d = 0
		for i in range(numScans):
			start = time.time()
			while True:
				if time.time() - start > 0.001:
					break	
			d += self.distance
		return d/numScans

	# SCANS THE ROOM 
	def scan(self):
		dur_scan = 0.11									# duration of turns during scan
		numScans = 15									# number of scans done by car
		scanDist = []									# list to store the scanned distances
		# eval("self.{0}()".format(fun))   				# call function to set speed

		# get car in initial position for scan
		self.turnLeft()
		self.wait(0.55)

		# start scanning
		for i in range(numScans):
			self.turnRight()
			self.wait(dur_scan)
			scanDist.append(self.measureDist(50))

		# choose direction
		maxPos = scanDist.index(max(scanDist)) 
		self.turnLeft()
		self.wait(dur_scan*(numScans-maxPos-1))			# turns car to max distance position
		self.stopCar()
		self.wait(1)


if __name__ == '__main__':
	autonomy().runner()

# hello