#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

vec_d1 = []

mem = []
with open("/home/edward/.static_mapping_log/static_mapping_.log.2020-11-26.23:56:20", "r") as file:
    lines = file.readlines()
    for line in lines:
        strs = line.split(",")
        mem.append(float(strs[1]))
file.close()

plt.title('Mem')
# Create a figure instance
fig = plt.figure(1, figsize=(9, 6))
# Create an axes instance
ax = fig.add_subplot(111)
bp = ax.plot(mem)
plt.show()
