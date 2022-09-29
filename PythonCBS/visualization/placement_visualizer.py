#This program is used to visualize the placement of chunks on the factory floor

"""
Inputs: Size of space in floor tiles (2 grid squares per floor tile), location
of all chunks, and which job each chunk belongs to.

Output: a graphic in a new window
"""

import math
from PIL import Image, ImageDraw

# def placement_vis(floor_size, chunk_positions, chunk_job):
    # job_number = max(chunk_job)
    
    
    
floor_size = [8,6]

chunk_positions = [[0,5],[1,5],[13,1],[14,1],[5,3],[5,2],[12,4],[12,5],\
                   [0,4],[1,4],[13,2],[14,2],[6,3],[6,2],[13,4],[13,5],\
                   [0,3],[1,3],[13,3],[14,3],[7,3],[7,2],[14,4],[14,5],]

chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]

# placement_vis(floor_size, chunk_positions, chunk_job)    

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
    min_x = min(x_positions)
    max_x = max(x_positions)
    min_y = min(y_positions)
    max_y = max(y_positions)
        
    
    