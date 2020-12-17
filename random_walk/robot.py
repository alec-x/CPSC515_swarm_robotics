# Based off robot in Sebastian Thrun's AI for Robotics course 
# https://classroom.udacity.com/courses/cs373

from math import cos, sin, pi
from scipy.stats import levy
from scipy.stats import cauchy
import random

class robot:
    # --------
    # init: 
    #   creates robot and initializes location to 0, 0
    #

    def __init__(self, x_init = 0, y_init = 0, heading_init = 0, measurement_range = 30.0,
                 motion_noise = 1.0, measurement_noise = 1.0, landmarks = [], step_len = 1.0):
        self.x = x_init
        self.y = y_init
        self.measurement_range = measurement_range
        self.heading = heading_init
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise
        self.landmarks = landmarks
        self.num_landmarks = 0
        self.step_len = step_len
        self.steps_remaining = 0

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
        for i in range(self.num_landmarks):
            dx = self.landmarks[i][0] - self.x + self.rand() * self.measurement_noise
            dy = self.landmarks[i][1] - self.y + self.rand() * self.measurement_noise    
            if self.measurement_range < 0.0 or abs(dx) + abs(dy) <= self.measurement_range:
                Z.append([i, dx, dy])
        return Z

    def collision(self):
        return False
    #
    # print robot location, heading, and num steps
    #

    def __repr__(self):
        return 'Robot: [x=%.5f y=%.5f heading=%.5f steps=%s]'  % \
            (self.x, self.y, self.heading, self.steps_remaining)

    def update(self):
        collided = self.collision()
        if(collided):
            return True
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