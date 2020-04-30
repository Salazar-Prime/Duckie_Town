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
		self.distance_flag = 0
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

			## Place code here
			self.start = time.time()
			print(self.start)

			self.path(self.start)

			
			##Leave these lines at the end
			self.publishMotors()
			self.publishServo()
#			self.publishLED()
			self.rate.sleep()

	def path(self,start):

		# specify your directions here: 
		# 1 = forward, 2 = left, and 
		# second column is time
		direction_1 = np.array([[1,1],[2,0.55],
								[1,1],[2,0.55],
								[1,1],[2,0.55],
								[1,1],[2,0.55] ])
		
		# only run once
		if self.flag == 0: 

			for i in direction_1:

				# set speeds
				if i[0] == 1:
					self.leftSpeed = 0.7 
					self.rightSpeed = 0.7
				elif i[0] == 2:
					self.leftSpeed = -0.7
					self.rightSpeed = 0.7
				self.publishMotors()

				# make it motors run for stiplated time
				start = time.time()
				while True:
					print(self.flag)
					print('distance', self.distance<0.4)

					# update distance flag if objetc is detected
					if self.distance < 0.4:
						self.distance_flag = 1
						break
					if time.time() - start > i[1]:
						break
				
				# stop car for obstacle and break out of the program
				if self.distance_flag == 1:
					self.flag = 1
					self.leftSpeed = 0
					self.rightSpeed = 0
					self.publishMotors()
					break

				# just a 0.5s break betweeen each direction to prevent slip
				self.leftSpeed = 0
				self.rightSpeed = 0
				self.publishMotors()
				start = time.time()
				while True:
					if time.time() - start > 0.5:
						break
		
		# # stop the motors
		# # comment out for infinite execution - part 2
		# self.flag = 1
		# self.leftSpeed = 0
		# self.rightSpeed = 0
		# self.publishMotors()

	def pid(self, dist):
		# ki = 0.001
		ki = 0.001
		kp = 1

		# integral clamping
		if self.integral > 2: 
			self.integral = 2
		elif self.integral < -2:
			self.integral = -2

		# pid control
		error = dist - self.setpoint
		self.integral += (error / 100)
		speed =  kp * error + ki * self.integral 

		# speed clamping
		if speed > 0.7:
			speed = 0.7
		elif speed < -0.7:
			speed = -0.7
		return speed


if __name__ == '__main__':
	autonomy().runner()

