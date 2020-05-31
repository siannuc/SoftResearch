# -*- coding: utf-8 -*-
"""
Created on Fri May 29 16:00:33 2020

@author: Steve Iannucci
"""


""" 
	import necessary packages
"""

import pyrealsense2 as rs
import numpy as np 
import cv2
from matplotlib import pyplot as plt 
import math
import sys
import platform
import csv

"""
	declare variables
"""

# Set number of measured points
n = 6

# picture properties
rows = 360
columns = 640
f_rate = 30 

# colors
blue = (255,0,0)
green = (0, 255, 0)
red = (0,0,255)
yellow = (200, 250, 0)

# set program steps
step_1 = False
step_2 = False
step_3 = False
step_4 = False

# variables
klick_counter = 0
ix = 0
iy = 0
ch_poi = np.zeros((n,2))  	#chosen point
koord = np.zeros((n,3))		# coordinates
corner_picture = np.zeros((n,16,16,3), dtype = np.uint8) # image stack
depth_point = np.zeros((n,3)) # output data


"""
	FUNCTIONS
"""

def get_mouse_position(event, x, y, flags, param):
	global ix, iy, klick_counter
	if event == cv2.EVENT_LBUTTONDOWN:
		ix, iy = x, y
		klick_counter +=1
		print ("Click Registered")

def good_features_to_track(src_color):
	src_color = src_color.reshape((16,16,3))
	gray = cv2.cvtColor(src_color, cv2.COLOR_BGR2GRAY)
	corners = cv2.goodFeaturesToTrack(gray, 1, 0.01, 10)
	corners = np.int0(corners)
	for i in corners:
		x_corner, y_corner = i.ravel()
	return x_corner, y_corner

"""
WRITE SYSTEM PROPERTIES
"""
print ("Python Version: "+sys.version)
print ("Windows Version: "+platform.platform())


"""
DEFINE STREAM PROPERTIES
"""
#streams
pipeline = rs.pipeline() 														#create pipeline
config = rs.config()															#create a configuration
config.enable_stream(rs.stream.color, columns, rows, rs.format.bgr8, f_rate)	#get the color stream
config.enable_stream(rs.stream.depth, columns, rows, rs.format.z16, f_rate)		#get the depth stream
config.enable_stream(rs.stream.infrared, 1, columns, rows, rs.format.y8, f_rate) #get left IR streams
config.enable_stream(rs.stream.infrared, 2, columns, rows, rs.format.y8, f_rate) #get right IR streams

# start streaming
profile = config.resolve(pipeline)
profile = pipeline.start(config)

# get the depth sensor scale
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ",depth_scale)

# create an align object
align_to = rs.stream.color
align = rs.align(align_to)

"""
MAIN PROGRAM
"""

# Create variables

def main():
	# get global variables = variables that can be changed in main function
	global step_1, step_2, step_3, step_4, klick_counter,ch_poi, koord, corner_picture, depth_point_1
	# start the main - try
	try:
		while True:

			frames = pipeline.wait_for_frames()
			depth_frame = frames.get_depth_frame()
			color_frame = frames.get_color_frame()
			aligned_frames = align.process(frames)
			aligned_depth_frame = aligned_frames.get_depth_frame()
			aligned_color_frame = aligned_frames.get_color_frame()

			# validate frames
			if not aligned_depth_frame or not aligned_color_frame:
				continue

			ir_left_frame = frames.get_infrared_frame(1)
			ir_right_frame = frames.get_infrared_frame(2)
			depth_image = np.asanyarray(aligned_depth_frame.get_data())
			color_image = np.asanyarray(aligned_color_frame.get_data())
			

			# intrinsics and extrinsics
			depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
			color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
			depth_to_color_intrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

			delta = 8 # spacing on the 
			try:
				
				if cv2.waitKey(1) == ord('k') or step_1 == True:
					print("Select Points")
					step_1 = True
					cv2.setMouseCallback('Align Example', get_mouse_position)
					for i in range(0,n):
						cv2.rectangle(color_image, (int(ch_poi[i][0]) - delta, int(ch_poi[i][1]) - delta), (int(ch_poi[i][0]) + delta, int(ch_poi[i][1]) + delta), red, 0)

					# print ix and iy
					if klick_counter <= n:
						ch_poi[klick_counter-1][0] = ix
						ch_poi[klick_counter-1][1] = iy
						if klick_counter == n:
							klick_counter = 0
							step_1 = False
							step_2 = True
			except:
				print ("error at second try")

			try:
				if step_2 == True:

					for i in range(0,n):
						corner_picture[i][:][:][:] = color_image[int(ch_poi[i][1]) - delta: int(ch_poi[i][1]) + delta, int(ch_poi[i][0]) - delta: int(ch_poi[i][0]) + delta]

					# print(corner_picture_0.shape)
					for i in range(0,n):
						koord[i][0], koord[i][1] = good_features_to_track(corner_picture[i][:][:][:])

					for i in range(0,n):
						for j in range(0,2):
							koord[i][j] = ch_poi[i][j] - delta + koord[i][j]
				
					for i in range(0,6):
						cv2.rectangle(color_image, (int(koord[i][0]) - delta, int(koord[i][1]) - delta), (int(koord[i][0]) + delta, int(koord[i][1])+delta), green, 0)

					step_3 = True
			except:
				print ("error at third try")

			try:
				if step_3 == True:
					for i in range(0,n):
						z = aligned_depth_frame.get_distance(int(koord[i][0]), int(koord[i][1]))
						
						# save z
						if z > 0:
							koord[i][2] = z

					step_3 = False
					step_4 = True
			except:
				print ("Error at the fourth try")

			try:
				if step_4 == True:
					for i in range(0,6):
						depth_point[i][:] = rs.rs2_deproject_pixel_to_point(depth_intrin, [koord[i][0],koord[i][1]], koord[i][2])
					
					# Save Data in .CSV file
					with open("C:/Users/Steve Iannucci/Documents/SoftResearch/iteracc.csv", 'a') as csvfile: 
						filewriter = csv.writer(csvfile, delimiter = ',', quoting=csv.QUOTE_NONE, lineterminator = '\n')
						filewriter.writerow(depth_point.ravel())
						print('Saving')

					step_4 = False
			except:
				print ("error at fifth try")

			try:
				cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
				cv2.imshow('Align Example', color_image)

			except:
				print ("error occured during streaming")	

			if cv2.waitKey(1) == 27: # when 0. program updates whenever button is pressed. when 1 program closes when escape is held
				break
	except:
		print ("error occured in first try")
	finally:
		cv2.destroyAllWindows()
		pipeline.stop()
		print('Pipeline Stopped')


if __name__ == '__main__':
	main()