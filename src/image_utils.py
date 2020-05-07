import cv2
import numpy as np
import math

# import the necessary packages for visual logging
from logging import FileHandler
from vlogging import VisualRecord
import logging

flag = 0
logs = []
log_flag = "edges"  # ['stoplight', 'ducks', 'heading', 'edges']


def imageProcessQueue(frame):
	global flag, logs
	logs = [] # [np_lanes, steering_angle, LR_lane_indicator, ducks_flag]

	frame = rescale_frame(frame)

	edges, ducks_area, stoplight = detect_edges(frame)
	edges = region_of_interest(edges)
	line_segments = detect_line_segments(edges)
	lane_lines, lane_flag = average_slope_intercept(frame, line_segments)
	logs.append(compute_steering_angle(frame, lane_lines))
	logs.append(lane_flag)
	logs.append(ducks_area)
	logs.append(stoplight) # 1=red, 2=green
	final_frame = display_lines(frame, lane_lines, logs[1])
		
	# debugging
	if flag != 2 and log_flag == "heading":
		show_image("heading overlay", final_frame)	
		flag += 1


	return final_frame, logs

def rescale_frame(frame, percent=40):
	width = int(frame.shape[1] * percent/ 100)
	height = int(frame.shape[0] * percent/ 100)
	dim = (width, height)
	return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)


def detect_edges(frame):
	global flag, log_flag

	# filter for blue lane lines
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# blue tape home
	lower_blue = np.array([85.2,47.7,36.9])
	upper_blue = np.array([148.9,255,199.9])
	mask = cv2.inRange(hsv, lower_blue, upper_blue)

	# detect edges
	edges = cv2.Canny(mask, 200, 400)

	# detect ducks
	ducks_area = detect_ducks(hsv)

	# detect ducks
	stoplight = detect_stoplight(hsv)

	# debugging
	if flag != 2 and log_flag == "edges":
		# cv2.imwrite("./calibrating.png",frame)
		show_image("Detect_Edges", [frame, hsv, mask, edges])
		flag += 1

	return edges, ducks_area, stoplight

def detect_ducks(hsv):
	global flag

	# detect ducks
	lower_yellow = np.array([5.3,67.57,164.9])
	upper_yellow = np.array([43.5,237.1,255])
	mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

	# debugging
	if flag != 2 and log_flag == "ducks":
		show_image("Showing ducks", [hsv, mask])
		flag += 1

	return np.count_nonzero(mask)

def detect_stoplight(hsv):
	global flag

	# detect red_light
	lower_red = np.array([164.68,91.29,158.86])
	upper_red = np.array([173.8,255,255])
	mask_red = cv2.inRange(hsv, lower_red, upper_red)

	# # detect green_light
	# lower_green = np.array([46,37.74,170.85])
	# upper_green = np.array([72.85,255,255])
	# mask_green = cv2.inRange(hsv, lower_green , upper_green)

	# detect green_light
	lower_green = np.array([43.3,37.74,105.3])
	upper_green = np.array([72.85,255,255])
	mask_green = cv2.inRange(hsv, lower_green , upper_green)

	red_pixels = np.count_nonzero(mask_red)
	green_pixels = np.count_nonzero(mask_green)
	# print("red pixels: %d green_pixels %d" % (red_pixels, green_pixels))
	# debugging
	if flag != 2 and log_flag == "stoplight":
		# cv2.imwrite("./mask.png",mask)
		show_image("Showing light", [hsv, mask_red, mask_green])
		flag += 1
	
	if red_pixels > 3800:
		return 1
	elif green_pixels > 3500:
		return 2
	else:
		return 0

def region_of_interest(edges):
	height, width = edges.shape
	mask = np.zeros_like(edges)

	# only focus bottom half of the screen
	polygon = np.array([[
		(0, height * 1 / 2),
		(width, height * 1 / 2),
		(width, height),
		(0, height),
	]], np.int32)

	cv2.fillPoly(mask, polygon, 255)
	cropped_edges = cv2.bitwise_and(edges, mask)
	return cropped_edges

def detect_line_segments(cropped_edges):
	# tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
	rho = 1  # distance precision in pixel, i.e. 1 pixel
	angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
	min_threshold = 40  # minimal of votes
	# line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
	# 								np.array([]), minLineLength=40, maxLineGap=50)
	line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
									np.array([]), minLineLength=60, maxLineGap=50)

	return line_segments

def average_slope_intercept(frame, line_segments):
	"""
	This function combines lin			# turn_speed_here = 0.35
			# if self.logs[0] == 1:
			# 	if self.logs[2][0] == 1:
			# 		self.leftSpeed = turn_speed_here
			# 		self.rightSpeed = turn_speed_here - 0.1
			# 		self.integral = 0
			# 		self.prev_error = 0
			# 	if self.logs[2][1] == 1:
			# 		self.leftSpeed = turn_speed_here - 0.1
			# 		self.rightSpeed = turn_speed_here
			# 		self.integral = 0
			# 		self.prev_error = 0

e segments into one or two lane lines
	If all line slopes are < 0: then we only have detected left lane
	If all line slopes are > 0: then we only have detected right lane
	"""

	lane_flag = [0,0]
	lane_lines = []
	if line_segments is None:
		logging.info('No line_segment segments detected')
		return lane_lines, lane_flag
	# print(frame.shape)
	height, width, _ = frame.shape
	left_fit = []
	right_fit = []

	boundary = 1/6    #### orignally 1/3
	left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
	right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

	for line_segment in line_segments:
		for x1, y1, x2, y2 in line_segment:
			if x1 == x2:
				logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
				continue
			fit = np.polyfit((x1, x2), (y1, y2), 1)
			slope = fit[0]
			intercept = fit[1]
			if slope < 0:
				if x1 < left_region_boundary and x2 < left_region_boundary:
					left_fit.append((slope, intercept))
			else:
				if x1 > right_region_boundary and x2 > right_region_boundary:
					right_fit.append((slope, intercept))

	left_fit_average = np.average(left_fit, axis=0)
	if len(left_fit) > 0:
		lane_lines.append(make_points(frame, left_fit_average))

	right_fit_average = np.average(right_fit, axis=0)
	if len(right_fit) > 0:
		lane_lines.append(make_points(frame, right_fit_average))

	
	if len(left_fit) > 0:
		lane_flag[0] = 1
	if len(right_fit) > 0:
		lane_flag[1] = 1
	return lane_lines, lane_flag


def make_points(frame, line):
	height, width, _ = frame.shape
	slope, intercept = line
	y1 = height  # bottom of the frame
	y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

	# bound the coordinates within the frame
	x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
	x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
	return [[x1, y1, x2, y2]]


def compute_steering_angle(frame, lane_lines):
	""" Find the steering angle based on lane line coordinate
		We assume that camera is calibrated to point to dead center
	"""
	global logs
	logs.append(len(lane_lines))
	
	if len(lane_lines) == 0:
		logging.info('No lane lines detected, do nothing')
		return -90

	height, width, _ = frame.shape
	if len(lane_lines) == 1:
		## decide what to do if only one line is detected
		logging.debug('Only detected one lane line, just follow it. %s' % lane_lines[0])
		x1, _, x2, _ = lane_lines[0][0]
		x_offset = x2 - x1
	elif len(lane_lines) == 2:
		_, _, left_x2, _ = lane_lines[0][0]
		_, _, right_x2, _ = lane_lines[1][0]
		camera_mid_offset_percent = 0 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
		mid = int(width / 2 * (1 + camera_mid_offset_percent))
		mid = width / 2 * (1 + camera_mid_offset_percent)
		x_offset = (left_x2 + right_x2) / 2 - mid

	# find the steering angle, which is angle between navigation direction to end of center line
	# y_offset = int(height / 2)
	y_offset = height / 2

	angle_to_mid_radian = math.atan2(x_offset, y_offset)  # angle (in radian) to center vertical line
	# angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
	angle_to_mid_deg = angle_to_mid_radian * 180.0 / math.pi  # angle (in degrees) to center vertical line
	steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

	logging.debug('new steering angle: %s' % steering_angle)
	return steering_angle


##########################################################################
##																		##
## functions pertaining to visual debugging								##
##																		##
##########################################################################


def display_lines(frame, lines, steering_angle, line_color=(0, 255, 0), line_width=2):
	line_image = np.zeros_like(frame)
	if lines is not None:
		for line in lines:
			for x1, y1, x2, y2 in line:
				cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
	line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
	# show_image("lane overlay", line_image)
	return display_heading_line(line_image, steering_angle)


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
	heading_image = np.zeros_like(frame)
	height, width, _ = frame.shape

	# figure out the heading line from steering angle
	# heading line (x1,y1) is always center bottom of the screen
	# (x2, y2) requires a bit of trigonometry

	# Note: the steering angle of:
	# 0-89 degree: turn left
	# 90 degree: going straight
	# 91-180 degree: turn right 
	steering_angle_radian = steering_angle / 180.0 * math.pi
	x1 = int(width / 2)
	y1 = height
	x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
	y2 = int(height / 2)

	cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
	heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
	return heading_image

def display_ducks(frame, ducks, line_color=[255, 0, 0]):
	# duck_image = np.zeros_like(frame)
	# duck_image[np.where(ducks==[255])] = line_color
	# ducks[np.where((ducks==[255]).all(axis=1))] = []
	# ducks[np.where((ducks==[0]).all(axis=1))] = [255]
	# alpha = 0.4
	# out = frame.copy()
	# duck_image = frame.copy()
	duck_image = np.zeros_like(frame)
	print("here1")
	duck_image[ducks] = line_color
	print("here2")

	# out = cv2.addWeighted(duck_image, alpha, out, 1 - alpha, 0, out)
	# duck_image = cv2.addWeighted(frame,0.4,ducks,0.1,0)
	duck_image = cv2.addWeighted(frame, 0.8, duck_image, 1, 1)
	return duck_image


def show_image(msg, frame):
	# open the logging file
	logger = logging.getLogger("visual_logging_example")
	fh = FileHandler("demo.html", mode = "w")

	# set the logger attributes
	logger.setLevel(logging.DEBUG)
	logger.addHandler(fh)

	logger.debug(VisualRecord(msg, frame, fmt = "png"))
