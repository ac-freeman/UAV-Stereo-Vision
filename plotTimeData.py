# short script to plot time data for each frame
# saves graph to png and eps
# looks for data in timeData.txt
# June 11, 2017

import numpy as py
import matplotlib.pyplot as plt

with open("timeData.txt") as timeData:
    data = timeData.readlines()
    data = [x.strip() for x in data]
# read in data line by line

for i in range(len(data)):
    data[i] = float(data[i])
# store data as floats in array

# plot data
plt.plot(range(1, len(data)+1), data, 'ro-')
plt.title('Processing Time Data')
plt.ylabel('Processing Time (s)')
plt.xlabel('Frame Number')
plt.savefig('timeData.png')
plt.savefig('timeData.eps')
plt.show()
