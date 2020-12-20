# Based off robot in Sebastian Thrun's AI for Robotics course 
# https://classroom.udacity.com/courses/cs373

from math import cos, sin, pi
from scipy.stats import levy
from scipy.stats import cauchy
from scipy.spatial.distance import euclidean
import numpy as np
import random

class robot:
    # --------
    # init: 
    #   creates robot and initializes location to 0, 0
    #

    def __init__(self, x_init = 0, y_init = 0, heading_init = 0, sense_range = 10,
                 motion_noise = 0.05, turn_noise = 0.02, measurement_noise = 1.0, step_len = 1.0,
                 landmarks = [], world = np.asarray([]), occ_grid = np.asarray([])):
        self.x_real = x_init
        self.y_real= y_init
        self.heading_real = heading_init
        self.x_sense = self.x_real
        self.y_sense = self.y_real
        self.heading_sense = self.heading_real
        self.sense_range = sense_range
        self.motion_noise = motion_noise
        self.turn_noise = turn_noise
        self.measurement_noise = measurement_noise
        self.step_len = step_len
        self.steps_remaining = 0
        self.landmarks = landmarks
        self.world = world
        self.occ_grid = occ_grid

    def rand(self):
        return float(random.random() * 2.0 - 1.0)

    #
    # move: attempts to move robot by_realstep_len in direction of heading.
    #

    def step(self):
        x = self.x_real + self.step_len*cos(self.heading_real) + self.rand() * self.motion_noise
        y = self.y_real+ self.step_len*sin(self.heading_real) + self.rand() * self.motion_noise

        # Dead reckoning increments no matter what
        self.x_sense = self.x_sense + self.step_len*cos(self.heading_sense)
        self.y_sense = self.y_sense + self.step_len*sin(self.heading_sense)
        if(self.world[int(x)][int(y)] == 1): # fail to step if on wall
            # Robot doesnt move because hit wall
            return False
        else:
            self.x_real = x
            self.y_real= y
            return True
    
    def turn(self, angle):
        heading = self.heading_real + angle + self.rand() * self.turn_noise
        
        if(0):
            return False
        else:
    
            self.heading_real = heading
            self.heading_sense = self.heading_sense + angle
            
            return True

    #
    # sense_landmarks: returns x- and y- distances to landmarks
    #

    def sense_landmarks(self):
        Z = []
        for i in range(len(self.landmarks)):
            dx = self.landmarks[i][0] - self.x_real + self.rand() * self.measurement_noise
            dy = self.landmarks[i][1] - self.y_real+ self.rand() * self.measurement_noise    
            if self.sense_range < 0.0 or abs(dx) + abs(dy) <= self.sense_range:
                Z.append([i, dx, dy])
        return Z

    def sense_proximity(self):
        x_lower = round(self.x_real) - self.sense_range
        x_upper = round(self.x_real) + self.sense_range + 1
        y_lower = round(self.y_real) - self.sense_range
        y_upper = round(self.y_real) + self.sense_range + 1
        neighborhood = self.world[x_lower:x_upper, y_lower:y_upper]
        if np.sum(neighborhood) > 0:
            return True, neighborhood
        return False, neighborhood
    #
    # print robot location, heading, and num steps
    #

    def __repr__(self):
        return 'Robot: [x=%.5f y=%.5f heading=%.5f steps_left=%s]'  % \
            (self.x_real, self.y_real, self.heading_real, self.steps_remaining)

    def update(self):
        # Sense
        landmarks = self.sense_landmarks()
        collided, prox = self.sense_proximity()

        # Record
        x_lower = round(self.x_sense) - self.sense_range
        x_higher = round(self.x_sense) + self.sense_range + 1
        y_lower = round(self.y_sense) - self.sense_range
        y_higher = round(self.y_sense) + self.sense_range + 1

        # Increment occ_grids for 0 and 1 by_realdetected type in grid points in sensor range
        self.occ_grid[1][x_lower:x_higher,y_lower:y_higher] += prox.astype(int)
        self.occ_grid[0][x_lower:x_higher,y_lower:y_higher] += np.logical_not(prox).astype(int)

        # Move
        if(collided):
            self.turn(pi) # turn 180 degrees

            self.steps_remaining += 3
            self.step() # TODO: fix_real robot stuck in wall bug, then remove auto steps
                        # might be fixed by_realimplementing localization....

        elif self.steps_remaining <= 0:
            self.steps_remaining = int(levy.rvs())
            angle = cauchy.rvs()
            self.turn(angle)
            return True
        else:
            self.step()
            self.steps_remaining -= 1
            return True
        