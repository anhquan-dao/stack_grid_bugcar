#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import itertools
import cv2
import imutils
import time 

layer = list()

for i in range(5):
    layer.append(np.random.randint(0, 100, size=(40000)))


secondary_grid = [None] * 5
main_map_img = np.zeros(shape = (400,400,1), dtype = np.float32)

main_resolution = 0.1
origin_x = 1
origin_y = 1



for i in range(len(secondary_grid)):
    secondary_grid[i] = np.reshape(layer[i],(200,200))
    secondary_grid[i] = secondary_grid[i].astype(np.float32)
    #print("Reshape speed " + str(1/(time.time()-dt)))

    tf_matrix_2d = cv2.getRotationMatrix2D((0,200), 30, 1.0)
    new_size = int(np.sqrt(secondary_grid[i].shape[0]**2 + secondary_grid[i].shape[1]**2))

    secondary_grid[i] = cv2.flip(cv2.warpAffine(cv2.flip(secondary_grid[i],0), tf_matrix_2d, (400,400)),0)

    
    #print("transform time " + str(1/(time.time()-dt)))
dt = time.time()
main_map_array = np.max(main_map_img,axis = 2)

layer = list()
main_map_array = main_map_array.astype(np.uint8)
dt = time.time()
layer = main_map_array.flatten().tolist()
print(time.time()-dt)
'''
dt = time.time()
for i in range(main_map_array.shape[-1]):
    layer.extend(main_map_array[:,i])
print(time.time()-dt)
'''
print("transform time " + str((time.time()-dt)))
cv2.imshow("yo",secondary_grid[0])
cv2.waitKey(0)

