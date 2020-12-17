from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from robot import robot

num_bots = 10 # number of robots
init_spacing = 50 # initial spacing (mm)

obs_len = 100 # obstacle dimensions
obs_width = 50

world_x = 550 # occupancy grid with 1 mm resolution
world_y = 600

def animate_loop(i):
    ax.clear()

    for bot in robots:
        bot.update()
    x_bots = [bot.x for bot in robots]
    y_bots = [bot.y for bot in robots]
    ax.set_xlim(0,world_x)
    ax.set_ylim(0,world_y)
    ax.plot(x_bots, y_bots, 'bo')
    [ax.add_patch(rect) for rect in obstacles] 

robots = [robot(160 + i*25, init_spacing) for i in range(num_bots)]

obstacles = [] 
obs_dims = [(100,100),(100,400),(250,250),(400,100),(400,400)] 
[obstacles.append(patches.Rectangle(dims,obs_width,obs_len,facecolor='r')) for dims in dims]
obs_dims = [(r._x0, r._y0, r._x0 +r._width, r._y0 + r._height) for r in obstacles]
 
# create animation
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
animation = FuncAnimation(fig, func=animate_loop, interval=1000)

# start animation
plt.show()