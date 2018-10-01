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
ch_poi = np.zeros((2,2))  	#chosen point
koord = np.zeros((2,3))		# coordinates

"""
	FUNCTIONS
"""

def get_mouse_position(event, x, y, flags, param):
	global ix, iy, klick_counter
	if event == cv2.EVENT_LBUTTONDOWN:
		ix, iy = x, y
		klick_counter +=1
		print ("button clicked")

def good_features_to_track(src_color):
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
# config.enable_stream(rs.stream.color, columns, rows, rs.format.bgr8, f_rate)	#get the color stream
# config.enable_stream(rs.stream.depth, columns, rows, rs.format.z16, f_rate)		#get the depth stream
# config.enable_stream(rs.stream.infrared, 1, columns, rows, rs.format.y8, f_rate) #get left IR streams
# config.enable_stream(rs.stream.infrared, 2, columns, rows, rs.format.y8, f_rate) #get right IR streams

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

def main():
	# get global variables = variables that can be changed in main function
	global step_1, step_2, step_3, step_4, klick_counter,ch_poi, koord
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

			delta = 8
			try:

				if cv2.waitKey(1) == ord('k') or step_1 == True:
					print("Test2")
					step_1 = True
					cv2.setMouseCallback('Align Example', get_mouse_position)
					cv2.rectangle(color_image, (int(ch_poi[0][0]) - delta, int(ch_poi[0][1]) - delta), (int(ch_poi[0][0]) + delta, int(ch_poi[0][1]) + delta), red, 0)
					cv2.rectangle(color_image, (int(ch_poi[1][0]) - delta, int(ch_poi[1][1]) - delta), (int(ch_poi[1][0]) + delta, int(ch_poi[1][1]) + delta), red, 0)

					# print ix and iy
					if klick_counter <= 2:
						ch_poi[klick_counter-1][0] = ix
						ch_poi[klick_counter-1][1] = iy
						if klick_counter == 2:
							klick_counter = 0
							step_1 = False
							step_2 = True
			except:
				print ("error at second try")

			try:
				if step_2 == True:

					corner_picture_0 = color_image[int(ch_poi[0][1]) - delta: int(ch_poi[0][1]) + delta, int(ch_poi[0][0]) - delta: int(ch_poi[0][0]) + delta]
					corner_picture_1 = color_image[int(ch_poi[1][1]) - delta: int(ch_poi[1][1]) + delta, int(ch_poi[1][0]) - delta: int(ch_poi[1][0]) + delta]


					koord[0][0], koord[0][1] = good_features_to_track(corner_picture_0)
					koord[1][0], koord[1][1] = good_features_to_track(corner_picture_1)

					for i in range(0,2):
						koord[i][0] = ch_poi[i][0] - delta + koord[i][0]
						koord[i][1] = ch_poi[i][1] - delta + koord[i][1]

					cv2.rectangle(color_image, (int(koord[0][0]) - delta, int(koord[0][1]) - delta), (int(koord[0][0]) + delta, int(koord[0][1])+delta), green, 0)
					cv2.rectangle(color_image, (int(koord[1][0]) - delta, int(koord[1][1]) - delta), (int(koord[1][0]) + delta, int(koord[1][1])+delta), green, 0)

					step_3 = True
			except:
				print ("error at third try")

			try:
				zero = 0
				np.float64(zero)
				if step_3 == True:
					for i in range(0,2):
						z = aligned_depth_frame.get_distance(int(koord[i][0]), int(koord[i][1]))
						# print z
						if z > 0:
							koord[i][2] = z
					if koord[0][2] > 0 and koord[1][2] > 0:
						step_3 = False
						step_4 = True
			except:
				print ("Error at the fourth try")

			try:
				if step_4 == True:
					depth_point_1 = rs.rs2_deproject_pixel_to_point(depth_intrin, [koord[0][0], koord[0][1]], koord[0][2])
					depth_point_2 = rs.rs2_deproject_pixel_to_point(depth_intrin, [koord[1][0], koord[1][1]], koord[1][2])
					vector_12 = np.zeros((1,3))
					for i in range(0,3):
						vector_12[0][i] = depth_point_1[i] - depth_point_2[i]
					print ("the position of point 1 is " + str(depth_point_1))
					print ("the position of point 2 is " + str(depth_point_2))
					
					with open("C:/Users/Steve/Documents/GitHub/SoftResearch/Data/tester1.csv", 'w') as csvfile: 
						filewriter = csv.writer(csvfile, delimiter = ',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
						filewriter.writerow(str(depth_point_1[0][0]),str(depth_point_1[0][1]),str(depth_point_1[0][2]))
					# print ("the vector between the two points is " + str(vector_12))
					# dist_12 = math.sqrt(vector_12[0][0]**2 + vector_12[0][1]**2 + vector_12[0][2]**2)
					# dist_12 = dist_12 * 1000
					# dist_12 = int(dist_12)
					# print ("the 3D distance between points is " + str(dist_12) + "mm")
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


if __name__ == '__main__':
	main()