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
from geometry_msgs.msg import *
from fiducial_msgs.msg import *

# user imports
from image_utils import imageProcessQueue

class autonomy(object):

	def __init__(self):
		##USER PARAMETERS
		self.angle_setpoint = 90
		self.angle = 0
		self.fin_time = 0
		self.integral = 0
		self.prev_error = 0
		self.logs = [0, 0]
		self.u_turn = False
		self.u_turn_z = 1.0
		
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
		self.blobpub = rospy.Publisher('imageProc', Image, queue_size=10)


		#Create Subscriber callbacks
		def lineCallback(data):
			self.leftLine = data.leftLine
			self.midLine = data.midLine
			self.rightLine = data.rightLine

		def distanceCallback(data):
			self.distance = data.distance


		def imageProcessing(data):
			# st_time = time.time()
			try:
				frame=self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
			except CvBridgeError as e:
				print(e)

			##Place image processing code here!
			frame, self.logs = imageProcessQueue(frame)
			print(self.logs)
			self.angle = self.logs[1]

			self.blobpub.publish(self.bridge.cv2_to_imgmsg(frame,"bgr8"))	 
			# print("Time diff: %f\n Steering angle: %f\n" % (((st_time - self.fin_time)*1000), self.angle))
			# print(logs)
			# self.fin_time = st_time

		# callback for Fiducial_msgs
		def fiducialNav(data):
			# print(data)
			# For id = 6 - U-turn
			for m in data.transforms:
				# print("zzz",m.transform.translation.z)
				if m.fiducial_id == 6 and m.transform.translation.z < 1.5:
					self.u_turn = True
					# print("found 6")

		def speed_callback(data):
						## timing code
			# st_time = time.time()
			# print("Time diff: %f\n" % ((st_time - self.fin_time)*1000))
			# print(self.u_turn)
			turn_speed_here = 0.3
			turn_speed = self.pid()
			self.leftSpeed = (0.2 + turn_speed)*0.9
			self.rightSpeed = (0.2 - turn_speed)*0.9

			# logs[4] ---> 1 = red, 2 = green, 0 = no_light
			if self.logs[0] == 0 or self.logs[3] > 6000 or self.logs[4] == 1:
				self.leftSpeed = 0
				self.rightSpeed = 0
				self.integral = 0
				self.prev_error = 5
				print('stopping')
			elif self.u_turn == True:
				# print("here........................................")
				self.leftSpeed = 0.5
				self.rightSpeed = -0.5
				self.publishMotors()	
				print("before wait")
				self.wait(1.3)
				print("after wait")
				self.u_turn = False

			

			# if only one lane is detected
			# if self.logs[0] == 1:
			# 	if self.logs[2][0] == 1:
			# 		self.leftSpeed = turn_speed_here
			# 		self.rightSpeed = -turn_speed_here
			# 		self.integral = 0
			# 		self.prev_error = 0
			# 	if self.logs[2][1] == 1:
			# 		self.leftSpeed = -turn_speed_here
			# 		self.rightSpeed = turn_speed_here
			# 		self.integral = 0
			# 		self.prev_error = 0

			##Leave these lines at the end
			# print(self.leftSpeed)
			# print(self.rightSpeed)
			if self.u_turn == False:
				self.publishMotors()			

		#Subscribe to topics
		rospy.Subscriber('raspicam_node/image', Image, imageProcessing)
		rospy.Subscriber('imageProc', Image, speed_callback)
		rospy.Subscriber('distance', distance, distanceCallback)
		rospy.Subscriber('fiducial_transforms',FiducialTransformArray,fiducialNav)

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

		while not rospy.is_shutdown():

			## timing code
			# st_time = time.time()
			# # print("Time diff: %f\n" % ((st_time - self.fin_time)*1000))
			# turn_speed = self.pid()
			# self.leftSpeed = 0.18 + turn_speed
			# self.rightSpeed = 0.18 - turn_speed
			# self.fin_time = st_time
			# if self.logs[0] == 0:
			# 	self.leftSpeed = 0
			# 	self.rightSpeed = 0
			# 	self.integral = 0
			# ##Leave these lines at the end
			# self.publishMotors()
			# self.publishServo()
			self.rate.sleep()

	def pid(self):
		kp = 0.9
		ki = 0	
		kd = 0.5

		# if np.sum(self.logs[2]) == 1:
		# 	kp=0.9
		# 	ki= 0.001
		# 	kd=0
		# integral clamping
		if self.integral > 2: 
			self.integral = 2
		elif self.integral < -2:
			self.integral = -2

		# pid control
		error = (self.angle - self.angle_setpoint)/(self.angle_setpoint)
		self.integral += (error / 10)
		derivative = (error - self.prev_error) * 10
		speed =  kp * error + ki * self.integral + kd*derivative  
		self.prev_error = error
		speed /= 3
		# speed clamping
		if speed > 0.2:
			speed = 0.2
		elif speed < -0.2:
			speed = -0.2
		# print("Speed: ", speed/10000)

		if self.logs[0] == 1:
			if self.logs[2][0] == 1:
				speed=abs(speed)/1.1
			if self.logs[2][1] == 1:
				speed=-abs(speed)/1.1
		# print(speed)
		return(speed)

	def move(self):
				## copying.....................................................
		## ...............................................................
					# Calculate the error in the x and y directions
		max_speed = 0.2
		min_speed = 0.05
		kp_f = 0.8
		kd_f = 40

		# A fiducial was detected since last iteration of this loop
		if self.found:

			forward_error = self.arucoZ - 0.8
			lateral_error = self.arucoX

			# Calculate the amount of turning needed towards the fiducial
			# atan2 works for any point on a circle (as opposed to atan)
			# angular_error = math.degrees(math.atan2(self.arucoX, self.arucoZ))
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
	

if __name__ == '__main__':
	autonomy().runner()
