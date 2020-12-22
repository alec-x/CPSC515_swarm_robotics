# Taken from Sebastian Thrun's AifR course

# Slam takes five parameters as input and returns the vector mu. 
# This vector should have x, y coordinates interlaced, so for example, 
# if there were 2 poses and 2 landmarks, mu would look like:
#
#  mu =  array([[Px0], [Py0], 
#               [Px1], [Py1],
#               [Lx0], [Ly0],
#               [Lx1], [Ly1]])
#
# data - This is the data that is generated with the included
#        make_data function. You can also use test_data to
#        make sure your function gives the correct result.
#
# N -    The number of time steps.
#
# num_landmarks - The number of landmarks.
#
# motion_noise - The noise associated with motion. The update
#                strength for motion should be 1.0 / motion_noise.
#
# measurement_noise - The noise associated with measurement.
#                     The update strength for measurement should be
#                     1.0 / measurement_noise.

import random

######################################################

# --------------------------------
#
# print the result of SLAM, the robot pose(s) and the landmarks
#

def print_result(N, num_landmarks, result):
    print
    print('Estimated Pose')
    print(result[2*(N-1)], result[2*N-1])
    print()
    print('Estimated Landmarks:')
    for i in range(num_landmarks):
        print('    ['+ ', '.join('%.3f'%x for x in result[2*(N+i)]) + ', ' \
            + ', '.join('%.3f'%x for x in result[2*(N+i)+1]) +']')
    print()
    

# --------------------------------
#
# slam - retains entire path and all landmarks
#

############## ENTER YOUR CODE BELOW HERE ###################
import numpy as np

def slam(init, data, num_landmarks, motion_noise, measurement_noise):
    N = len(data) + 1
    dim = 2 * (N + num_landmarks)
    
    Omega = np.zeros((dim, dim))
    Omega[0][0] = 1.0
    Omega[1][1] = 1.0
    
    Xi = np.zeros((dim, 1))
    Xi[0][0] = init[0]
    Xi[1][0] = init[1]

    for k in range(len(data)):
        n = k * 2
        measurement = data[k][0]
        motion      = data[k][1]
        
        for i in range(len(measurement)):

            m = 2*(N + measurement[i][0])
            
            for b in range(2): # x and y
                Omega[n+b][n+b] +=  1.0 / measurement_noise
                Omega[m+b][m+b] +=  1.0 / measurement_noise
                Omega[n+b][m+b] += -1.0 / measurement_noise
                Omega[m+b][n+b] += -1.0 / measurement_noise
                Xi[n+b][0]      += -measurement[i][1+b] / measurement_noise
                Xi[m+b][0]      +=  measurement[i][1+b] / measurement_noise
                
        for b in range(4):
            Omega[n+b][n+b] += 1.0 / motion_noise
        for b in range(2):
            Omega[n+b  ][n+b+2] += -1.0 / motion_noise
            Omega[n+b+2][n+b  ] += -1.0 / motion_noise
            Xi[n+b  ][0]        += -motion[b] / motion_noise
            Xi[n+b+2][0]        +=  motion[b] / motion_noise
        
    mu = np.matmul(np.linalg.inv(Omega), Xi) 
        
    return mu