# SoftResearch

Python programs to obtain three dimensional positional values using a Intel RealSense D415.

Download RealSense API and python wrappers before running. Would recommoned following examples and making sure you can connect to the camera first. 

pip install pyrealsense2

To get the code to run, press and hold 'k' to start process for data recorder. Click points on CV stream. Data will be saved in .CSV file in location that can be set in code. Data is formatted (X1,Y1,Z1...Xn, Yn, Zn) per row, with each row saved for every frame. The number of points registered per run is set by the user.
