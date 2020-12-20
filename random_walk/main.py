from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os
from robot import robot
from random import random
import shutil


video          = False # animate process
video_interval = 10 # milliseconds/frame
num_frames     = 10001 # iterations to run for for non-video option

save           = False # Save intermediate occ_grids occasionally
save_rate      = 1000 # frames/(occ_grid recording)
plot_occ       = False # Plot aggregated occupancy grid

num_bots       = 1 # number of robots
s_range        = 20 # range of wall detection

rand_landmarks = False # Randomly scattered landmarks vs 4 at start
num_landmarks = 100 # Only used if rand_landmarks = True

obs_len        = 100 # obstacle dimensions
obs_width      = 50

world_x        = 550 # occupancy grid
world_y        = 600
border_t       = 2*s_range

print("\nLoading with settings:")
print(f"world size:     {world_x} x {world_y}")
print(f"num iterations: {num_frames}")
print(f"num robots:     {num_bots}")
print(f"sensor range:   {s_range}\n")

if(save):
    dir_name = 'data'
    if os.path.exists(dir_name):
        shutil.rmtree(dir_name)
    os.makedirs(dir_name)

def logic_loop(i):
    for bot in robots:
        bot.update()
    if i % save_rate == 0 and save:
        # TODO: Refactor the abs, shouldnt need it
        np.savetxt(r'data/0_occ_grid_' + str(num_frames - i) + '.csv', occ_grid[0], delimiter=',')
        np.savetxt(r'data/1_occ_grid_' + str(num_frames - i) + '.csv', occ_grid[1], delimiter=',')
    #if i % max(1,int(num_frames/100)) == 0:
        #print(f'progress: {num_frames - i}/{num_frames}', end="\r")

def animate_loop(i):
    ax.clear()
    ax.set_xlim(0,world_x + 2*border_t)
    ax.set_ylim(0,world_y + 2*border_t)
    
    for bot in robots:
        bot.update()
    x_bots = [bot.x_real for bot in robots]
    y_bots = [bot.y_real for bot in robots]
    ax.plot(x_bots, y_bots, 'go')
    """
    x_bots_dead = [bot.x_sense for bot in robots]
    y_bots_dead = [bot.y_sense for bot in robots]
    ax.plot(x_bots_dead, y_bots_dead, 'ro')
    """
    [ax.add_patch(rect) for rect in obstacles]

    if i % save_rate == 0 and save:
        np.savetxt(r'data/0_occ_grid_' + str(i) + '.csv', occ_grid[0], delimiter=',')
        np.savetxt(r'data/1_occ_grid_' + str(i) + '.csv', occ_grid[1], delimiter=',')   

# Initialize obstacles
obstacles = [] 
obs_dims = [(100,100),(100,400),(250,250),(400,100),(400,400)]

obs_dims = [(dims[0] + border_t, dims[1] + border_t) for dims in obs_dims]
[obstacles.append(patches.Rectangle(dims,obs_width,obs_len,facecolor='black')) for dims in obs_dims]
obs_dims = [(r._x0, r._y0, r._x0 +r._width, r._y0 + r._height) for r in obstacles]
 
# Initialize world

h_grid = np.pad(np.zeros((world_x,world_y)), border_t, mode='constant', constant_values=1)
occ_grid = np.zeros((2, h_grid.shape[0], h_grid.shape[1]))

for r in obs_dims:
    h_grid[r[0]:r[2], r[1]:r[3]] = 1 # votes for 0 or 1 by different robots/passes



# Initialize landmarks
landmarks = []
if rand_landmarks:
    [landmarks.append((i, random()*h_grid.shape[0], random()*h_grid.shape[1])) for i in range(num_landmarks)]
else:
    landmarks.append((0, 150 + border_t, 20 + border_t))
    landmarks.append((1, 150 + border_t, 80 + border_t))
    landmarks.append((2, 400 + border_t, 20 + border_t))
    landmarks.append((3, 400 + border_t, 80 + border_t))

# Initialize robots
robots = [robot(150 + border_t + random()*250, 50 + border_t, world=h_grid, occ_grid=occ_grid, landmarks=landmarks) for i in range(num_bots)]

print("Starting simulation")
if(video):
    # create animation
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    animation = FuncAnimation(fig, func=animate_loop, interval=video_interval)

    # start animation
    plt.show()
else:
    i = num_frames
    while(i > 0):
        logic_loop(i)
        i -= 1
    

# Plot occupancy grid
if(plot_occ):
    print("\n\nloading occupancy grid")
    occ_grid = occ_grid[1] - occ_grid[0]
    occ_grid[occ_grid > 0] = 1
    occ_grid[occ_grid < 0] = 0
    plt.figure(figsize=(6,6))
    plt.pcolor(occ_grid[::-1])
    plt.show()