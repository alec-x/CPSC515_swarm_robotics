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
                 motion_noise = 0.1, measurement_noise = 1.0, step_len = 1.0,
                 landmarks = [], world = np.asarray([]), occ_grid = np.asarray([])):
        self.x = x_init
        self.y = y_init
        self.sense_range = sense_range
        self.heading = heading_init
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise
        self.step_len = step_len
        self.steps_remaining = 0
        self.landmarks = landmarks
        self.world = world
        self.occ_grid = occ_grid

    def rand(self):
        return float(random.random() * 2.0 - 1.0)

    #
    # move: attempts to move robot by step_len in direction of heading.
    #

    def step(self):
        x = self.x + self.step_len*cos(self.heading) + self.rand() * self.motion_noise
        y = self.y + self.step_len*sin(self.heading) + self.rand() * self.motion_noise

        if(0): # think of move fail conditions later
            return False
        else:
            self.x = x
            self.y = y
            return True
    
    def turn(self, angle):
        heading = self.heading + angle + self.rand() * self.motion_noise
        
        if(0):
            return False
        else:
            self.heading = heading
            return True

    #
    # sense_landmarks: returns x- and y- distances to landmarks
    #

    def sense_landmarks(self):
        Z = []
        for i in range(len(self.landmarks)):
            dx = self.landmarks[i][0] - self.x + self.rand() * self.measurement_noise
            dy = self.landmarks[i][1] - self.y + self.rand() * self.measurement_noise    
            if self.sense_range < 0.0 or abs(dx) + abs(dy) <= self.sense_range:
                Z.append([i, dx, dy])
        return Z

    def sense_proximity(self):
        neighborhood = self.world[round(self.x) - self.sense_range:round(self.x) + self.sense_range + 1, 
                                  round(self.y) - self.sense_range:round(self.y) + self.sense_range + 1]
        if np.sum(neighborhood) > 0:
            return True, neighborhood
        return False, neighborhood
    #
    # print robot location, heading, and num steps
    #

    def __repr__(self):
        return 'Robot: [x=%.5f y=%.5f heading=%.5f steps_left=%s]'  % \
            (self.x, self.y, self.heading, self.steps_remaining)

    def update(self):
        # Sense
        landmarks = self.sense_landmarks()
        collided, prox = self.sense_proximity()

        # Move
        if(collided):
            self.turn(pi) # turn 180 degrees
            self.steps_remaining += 3
            self.step()
        elif self.steps_remaining <= 0:
            self.steps_remaining = int(levy.rvs())
            self.heading += cauchy.rvs()
            self.heading %= 2*pi
            self.turn(self.heading)
            return True
        else:
            self.step()
            self.steps_remaining -= 1
            return True
        
        # Record
        x_lower = round(self.x) - self.sense_range
        x_higher = round(self.x) + self.sense_range + 1
        y_lower = round(self.y) - self.sense_range
        y_higher = round(self.y) + self.sense_range + 1
        self.occ_grid[1][x_lower:x_higher,y_lower:y_higher] += prox.astype(int)
        self.occ_grid[0][x_lower:x_higher,y_lower:y_higher] += np.logical_not(prox).astype(int)

        