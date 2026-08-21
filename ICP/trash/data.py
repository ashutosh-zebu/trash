#!/usr/bin/env python3

# Point-to-point ICP minimizes the sum of squared Euclidean distances between each source point and its corresponding point in the target cloud.
# This is the most basic form and treats each correspondence as an isolated pair of points, without considering local surface structure.

import matplotlib.pyplot as plt
import numpy as np
import data_copy as ICP

def p(points):
    data_points = np.array(points)
    x = data_points[:,0]
    y = data_points[:,1]
    return x, y

source_points = np.random.rand(110, 2)
target_points = source_points*0.89 + 1
# source_points = [[0,5],[4,10],[8,3],[12,12],[16,18]]
# target_points = [[0,3],[4,8],[8,1],[12,10],[16,16]]

sx , sy = p(source_points)

tx , ty = p(target_points)

data = ICP.ICP(source=source_points, target=target_points,depth=0)
print(data)
for s, t, d in data:
    # Draw a line between source and its closest target
    plt.plot([s[0], t[0]], [s[1], t[1]], "o--", alpha=0.5)

plt.plot(sx,sy, linestyle = 'dotted') 
plt.plot(tx,ty, linestyle = 'dotted') 
# Add labels and grid
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Source Points Plot")
plt.legend()
plt.grid(True)

plt.show()
