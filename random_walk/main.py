from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from robot import robot
from random import random

num_bots = 10 # number of robots
s_range = 10 # range of wall detection

obs_len = 100 # obstacle dimensions
obs_width = 50

world_x = 550 # occupancy grid with 1 mm resolution
world_y = 600

border_t = 2*s_range

def animate_loop(i):
    ax.clear()
    ax.set_xlim(0,world_x + 2*border_t)
    ax.set_ylim(0,world_y + 2*border_t)
    
    for bot in robots:
        bot.update()
    x_bots = [bot.x for bot in robots]
    y_bots = [bot.y for bot in robots]

    ax.plot(x_bots, y_bots, 'bo')
    [ax.add_patch(rect) for rect in obstacles] 

# Initialize obstacles
obstacles = [] 
obs_dims = [(100,100),(100,400),(250,250),(400,100),(400,400)]

obs_dims = [(dims[0] + border_t, dims[1] + border_t) for dims in obs_dims]
[obstacles.append(patches.Rectangle(dims,obs_width,obs_len,facecolor='r')) for dims in obs_dims]
obs_dims = [(r._x0, r._y0, r._x0 +r._width, r._y0 + r._height) for r in obstacles]
 
# Initialize world

h_grid = np.pad(np.zeros((world_x,world_y)), border_t, mode='constant', constant_values=1)
occ_grid = np.zeros((2, h_grid.shape[0], h_grid.shape[1]))

for r in obs_dims:
    h_grid[r[0]:r[2], r[1]:r[3]] = 1 # votes for 0 or 1 by different robots/passes

# Initialize robots
robots = [robot(150 + border_t + random()*250, 50 + border_t, world=h_grid, occ_grid=occ_grid) for i in range(num_bots)]

# create animation
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
animation = FuncAnimation(fig, func=animate_loop, interval=10)

# start animation
plt.show()

