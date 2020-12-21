# Author
Alec Xu

# Swarm robotic mapping using random walk

This is a project created for CPSC 515 Fall 2020 at the University of British Columbia.

# Usage:
The following configurations are available along with their defaults in random_walk/main.py:
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

After configuring, run using python 3.8 with conda base.