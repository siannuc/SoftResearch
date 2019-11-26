# -*- coding: utf-8 -*-
"""
Created on Mon Nov 25 17:42:01 2019

@author: siannuc
"""

# read csv files for analysis
# import needed packages
import numpy as np
from matplotlib import pyplot as plt

inpath = "D:/Research/Data/"
filename = "actualtest_11_2some"# inlcude path and use forward slash not backslash
spath = inpath

# pull data from txt file assume csv
data = np.genfromtxt(inpath+filename+".csv", delimiter=',')
data = data[:,1:3]
# remove rows with nan from depth
data = data[~np.isnan(data).any(axis=1)]

# obtain length of actuator by subtracting recorded distance by distance to the datum
length = 762-data[:,0]

# plotting
plt.figure(1, figsize=(8.5,11)) # create figure
plt.scatter(data[:,1],length) # plot data
plt.xlabel('Pressure Reading (V)') #x axis lable
plt.ylabel('Length of Actuator (mm)') # y axis label

# set plot axis limits
axes = plt.gca()
axes.set_xlim([1,3.5])
axes.set_ylim([100,115])

# save and close png
plt.savefig('testplot.png')
plt.close

out_data = np.asarray([data[:,1],length])
np.savetxt(spath+filename+"_edited.csv", out_data, delimiter=",",fmt='%10.5f')
