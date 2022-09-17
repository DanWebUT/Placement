#This program is used to create the .yaml file of the floor

"""
As inputs, the program accepts a floor size (in number of floor tiles), robot
starting positions, robot goal positions, and any filled chunks (all based on 
grid position on floor)
"""

import math
import yaml

class sizes():
    #size multiplier for floor tiles to mm
    size_multiplier = 500 #mm
    #how large the robot is in the visualization
    robot_size = 5 #mm
    #not quite sure, but changes resolution of the visualization
    grid_size = 25

def conv_coord(grid_pos_x, grid_pos_y):
    mid = math.floor(sizes.size_multiplier/2)
    pos_x = grid_pos_x*sizes.size_multiplier
    pos_y = grid_pos_y*sizes.size_multiplier
    return(pos_x, pos_y, mid)


    
    
def make_floor(num_FT_x, num_FT_y, rob_start_pos, rob_goal_pos, filled_pos):
    num_robots = len(rob_start_pos)
    
    
    file_name = "AMBOTS_Floor"
    file_yaml = file_name + ".yaml"
    with open(file_yaml, 'w') as file:
        file.write("---\n")
        
        #robot goal positions
        file.write("GOAL:\n")
        for i in range(0,num_robots):
            (x, y, mid) = conv_coord(rob_goal_pos[i][0], rob_goal_pos[i][1])
            file.write("- !!python/tuple [" + str(x+mid) + ", " + \
                       str(y+mid) + "]\n")
                
        file.write("GRID_SIZE: " + str(sizes.grid_size) + "\n")
        file.write("RECT_OBSTACLES:\n")
        
        #Border
        (x, y, mid) = conv_coord(num_FT_x*2, num_FT_y*2)
        file.write("  0:\n")
        file.write("  - [" + str(0) + ", " + str(0) + "]\n")
        file.write("  - [" + str(x) + ", " + str(y) + "]\n")
        
        #disallowed areas in the middle of grid spaces
        num_spaces = ((num_FT_x)*(num_FT_y)*4)*2
        spaces_counter = 1
        for i in range(0,num_FT_x*2+1):
            for j in range(0,num_FT_y*2+1):
                file.write("  " + str(spaces_counter)+ ":\n")
                spaces_counter += 1
                (x, y, mid) = conv_coord(i, j)
                x_start = x - mid + sizes().robot_size*3
                y_start = y - mid + sizes().robot_size*3
                x_end = x + mid - sizes().robot_size*3
                y_end = y + mid - sizes().robot_size*3
                file.write("  - [" + str(x_start) + ", " + str(y_start) + "]\n")
                file.write("  - [" + str(x_end) + ", " + str(y_end) + "]\n")
        
        # #filled chunks
        # num_chunks = len(filled_pos)
        # for i in range(0,num_chunks):
        #     file.write("  " + str(i+num_spaces)+ ":\n")
        #     (x, y, mid) = conv_coord(filled_pos[i][0], filled_pos[i][1])
        #     x_start = x - mid
        #     y_start = y - mid
        #     x_end = x + mid
        #     y_end = y + mid
        #     file.write("  - [" + str(x_start) + ", " + str(y_start) + "]\n")
        #     file.write("  - [" + str(x_end) + ", " + str(y_end) + "]\n")
            
        #robot radius
        file.write("ROBOT_RADIUS: " + str(sizes().robot_size) + "\n")
        
        #robot starting positions
        file.write("START:\n")
        for i in range(0,num_robots):
            (x, y, mid) = conv_coord(rob_start_pos[i][0], rob_start_pos[i][1])
            file.write("- !!python/tuple [" + str(x+mid) + ", " + \
                       str(y+mid) + "]\n")
        
        # yaml.dump(dict_file, file)
    
rob_start_pos = [[0,0],[1,0]]
rob_goal_pos = [[1,1],[2,0]]
filled_pos = [[2,0]]
make_floor(2, 1, rob_start_pos, rob_goal_pos, filled_pos)