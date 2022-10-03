#This program is used to visualize the placement of chunks on the factory floor

"""
Inputs: Size of space in floor tiles (2 grid squares per floor tile), location
of all chunks, and which job each chunk belongs to.

Output: a graphic in a new window
"""

import math
import cv2
import numpy as np

def placement_vis(floor_size, chunk_positions, chunk_job):
    disp_mult = 100
    size_multiplier = 75
    start_offest = 100   
    
    #create window
    cv2.namedWindow('placement', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('placement', (1280, 720))
    canvas = np.ones((1080,1500,3), np.uint8)*255
    
    #calculate the total size of floor with multiplier
    x_end = floor_size[0]*2*size_multiplier + start_offest
    y_end = floor_size[1]*2*size_multiplier + start_offest
    
    #create floor grid
    cv2.rectangle(canvas, (start_offest, start_offest), (x_end,y_end), (0, 0, 255), thickness=3)
    #vertical lines
    for i in range(0,floor_size[0]*2):
        x = start_offest + i*size_multiplier
        cv2.line(canvas, (x,start_offest), (x, y_end), (0, 0, 255), thickness=3)
    #horizontal lines
    for i in range(0,floor_size[1]*2):
        y = start_offest + i*size_multiplier
        cv2.line(canvas, (start_offest, y), (x_end, y), (0, 0, 255), thickness=3)    
    
    job_number = max(chunk_job)[0] + 1
    for i in range(0,job_number):
        job_positions = []
        for j in range(0,len(chunk_positions)):
            if chunk_job[j][0] == i:
                job_positions.append(chunk_positions[j])
        x_positions = []
        y_positions = []        
        for j in range(0,len(job_positions)):
            x_positions.append(job_positions[j][0])
            y_positions.append(job_positions[j][1])
        min_x_grid = min(x_positions)
        max_x_grid = max(x_positions)+1
        min_y_grid = min(y_positions)
        max_y_grid = max(y_positions)+1
        
        min_x = min_x_grid*size_multiplier + start_offest
        min_y = min_y_grid*size_multiplier + start_offest
        max_x = max_x_grid*size_multiplier + start_offest
        max_y = max_y_grid*size_multiplier + start_offest
        
        #draw job
        random_color = np.round(np.random.rand(1,3)*255)
        cv2.rectangle(canvas, (min_x, min_y), (max_x, max_y), (int(random_color[0][0]), int(random_color[0][1]), int(random_color[0][2])), thickness=-1)
    
    
        
    #show and delete image
    cv2.imshow('placement',canvas)
    cv2.waitKey(0)
    cv2.destroyWindow('placement')


#TEST configuration 
# floor_size = [8,6]

# chunk_positions = [[0,5],[1,5],[13,1],[14,1],[5,3],[5,2],[12,8],[12,9],\
#                    [0,4],[1,4],[13,2],[14,2],[6,3],[6,2],[13,8],[13,9],\
#                    [0,3],[1,3],[13,3],[14,3],[7,3],[7,2],[14,8],[14,9],]

# chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]

# placement_vis(floor_size, chunk_positions, chunk_job)



