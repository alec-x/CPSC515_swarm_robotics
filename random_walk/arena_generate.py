import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from collections import deque

obs_len = 100 # obstacle dimensions
obs_width = 50

world_x = 550 # occupancy grid with 1 mm resolution
world_y = 600

obstacles = [] 
obs_dims = [(100,100),(100,400),(250,250),(400,100),(400,400)] 
[obstacles.append(patches.Rectangle(dims,obs_width,obs_len,facecolor='r')) for dims in obs_dims]
obs_dims = [(r._x0, r._y0, r._x0 +r._width, r._y0 + r._height) for r in obstacles]

h_grid = np.empty((world_y,world_x))

for r in obs_dims:
    h_grid[r[0]:r[2], r[1]:r[3]] = -1 

#1. first pass, get all the -1 location and put them in a queue
#2. pop the queue for each item put the left,right,up,down neighbours in the queue with accumulated distance
#3. put the item in a seen list to make sure you don't repeat items
#4. each item do the corner check to see if there's left/right/down/up and calculate distance accordingly
queue = np.where(h_grid == -1)
queue = [(queue[0][i], queue[1][i]) for i in range(len(queue[0]))]
d = deque(queue)
while(d):
    break
plt.imshow(h_grid, cmap='hot', interpolation='nearest')
plt.show()