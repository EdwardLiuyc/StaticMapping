#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
# from pylab import *
vec_d1 = []

vec_d8 = []
with open("/home/edward/src/mapping/pcd/error.csv", "r") as file:
    lines = file.readlines()
    for i in range(0, len(lines)):
        line = float(lines[i])
        vec_d8.append(line)
file.close()

plt.title('Gps&Path Error')
plt.ylabel("Error /m")
# Create a figure instance
fig = plt.figure(1, figsize=(9, 6))

# Create an axes instance
ax = fig.add_subplot(111)

# Create the boxplot
bp = ax.boxplot(vec_d8)
plt.xticks([1], [" "])

# Save the figure
fig.savefig('fig1.png', bbox_inches='tight')
plt.show()
