#This program is used to track the progress of the printing

"""
This program will track the steps of the printing process (printing and move
phase) and call the floor maker and cbs algorithm at each step of the process
"""
from typing import List, Tuple
import numpy as np
import floor_maker
import placement_visualizer
from cbs_mapf import planner
import yaml
import math
import time
import path_scrubber


class tuning_variables:
    #the time in minutes it takes for a robot to move from one grid space to another
    robot_speed = .2

class chunk_global_info:
    #Info on the chunk configuration to be printed that will not change during printing
    chunk_number = 24
    # chunk_print_time = np.round(np.random.rand(1,chunk_number)*10)
    chunk_print_time = [89., 82., 89., 84., 85., 80., 83., 87., 86., 81., 82., 87., 84., 64., 67., 78., \
            84., 94., 87., 49., 86., 89., 86., 83.]
    #all chunks on which a chunk is directly dependent
    
                          
    chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]
    
    initial_chunk_dependencies = [[],[0],[],[2],[],[4],[],[6],\
                          [0],[1,8],[2],[3,10],[4],[5,12],[6],[7,14],\
                          [8],[9,16],[10],[11,18],[12],[13,20],[14],[15,22]]

    
class chunk_configuration:
    #Info on the chunk configuration that will change between iterations (ie position) 
    job_directions = [0,2,1,1]
    print_direction = np.zeros(chunk_global_info().chunk_number)
    for i in range(0,chunk_global_info().chunk_number):
        print_direction[i] = job_directions[chunk_global_info().chunk_job[i][0]]     

    #chunk locations
    chunk_positions = [[0,5],[1,5],[13,1],[14,1],[5,2],[5,1],[9,4],[9,5],\
                      [0,4],[1,4],[13,2],[14,2],[6,2],[6,1],[10,4],[10,5],\
                      [0,3],[1,3],[13,3],[14,3],[7,2],[7,1],[11,4],[11,5],]
    
    #this still modifies the original list for some reason
    chunk_dependencies = chunk_global_info().initial_chunk_dependencies.copy()
    
    print_state = np.zeros(chunk_global_info().chunk_number)    
        

def queue(robot_starting_positions, chunk_info, floor_size):
    total_print_time = 0
    static_obstacles = []
    
    #loop while unprinted chunks
    # while min(chunk_configuration().print_state) <= 0:
    
    #Find all printable chunks
    printable_chunks = independent_chunks(chunk_global_info())
    
    #Find the robot position for all printable chunks
    printable_chunk_robot_positions = robot_chunk_positions(printable_chunks)
    
    #find robot goal positions
    #First decide which n chunks to print with n robots
    (robot_goal_positions, printing_chunks) =  min_cost(robot_starting_positions, printable_chunk_robot_positions, printable_chunks)
    
    #Find longest print time of current chunks
    printing_chunks_time = np.zeros(len(printing_chunks))
    for i in range(0,len(printing_chunks)):
        printing_chunks_time[i] = chunk_global_info().chunk_print_time[[printing_chunks[i]]]
    print_time = max(printing_chunks_time)
    
    #make floor for this configuration
    grid_size_multiplier = floor_maker.make_floor(floor_size[0], floor_size[1], robot_starting_positions, robot_goal_positions, static_obstacles)
    
    static_obstacles = vertices_to_obsts(static_obstacles)
    
    # Call cbs-mapf to plan
    load_scenario("AMBOTS_floor.yaml")
    planner_object = planner.Planner(GRID_SIZE, ROBOT_RADIUS, static_obstacles)
    # path = planner_object.plan(START, GOAL, debug=False)
    
    #find end position
    robot_ending_positions = []
    """
    Need to figure out data structure of path. Would like to record path for each move
    for a visualization, but the important thing is to record the length of time the 
    move stage takes
    """
    move_time = 15
    
    #mark printed chunks as printed
    for i in range(0,len(printing_chunks)):
        chunk_configuration().print_state[printing_chunks[i]] = 1
        static_obstacles.append(chunk_configuration().chunk_positions[printing_chunks[i]])
                    
    #remove dependencies of printed chunks
    chunk_global_info().chunk_dependencies = remove_dependencies(chunk_global_info().chunk_dependencies, printing_chunks)
    
    total_print_time = total_print_time + move_time + print_time
    
    #end loop
    
    
    return(robot_ending_positions, total_print_time)

def independent_chunks(chunk_global_info):
    #find which chunks have no dependencies
    
    printable_chunks = []
    for i  in range(0,chunk_global_info.chunk_number):
        if chunk_configuration().chunk_dependencies[i] == [] and chunk_configuration().print_state[i] == 0:
            printable_chunks.append(i)
            
    
    return(printable_chunks)

def robot_chunk_positions(printable_chunks):
    printable_chunk_robot_positions = []
    for i in range (0,len(printable_chunks)):
        if chunk_configuration().print_direction[printable_chunks[i]] == 0:
            x_pos = chunk_configuration().chunk_positions[printable_chunks[i]][0]
            y_pos = chunk_configuration().chunk_positions[printable_chunks[i]][1]-1
        if chunk_configuration().print_direction[printable_chunks[i]] == 1:
            x_pos = chunk_configuration().chunk_positions[printable_chunks[i]][0]+1
            y_pos = chunk_configuration().chunk_positions[printable_chunks[i]][1]
        if chunk_configuration().print_direction[printable_chunks[i]] == 2:
            x_pos = chunk_configuration().chunk_positions[printable_chunks[i]][0]
            y_pos = chunk_configuration().chunk_positions[printable_chunks[i]][1]+1
        if chunk_configuration().print_direction[printable_chunks[i]] == 3:
            x_pos = chunk_configuration().chunk_positions[printable_chunks[i]][0]-1
            y_pos = chunk_configuration().chunk_positions[printable_chunks[i]][1]
            
        printable_chunk_robot_positions.append([x_pos,y_pos])
        
    return(printable_chunk_robot_positions)
    

def min_cost(robot_positions, printable_chunk_robot_positions, printable_chunks, robot_schedules):
    # Greedy algorithm to assign chunks to all the agents
    # Just needs to output the robot positions
    sqdist = lambda x, y: (x[0]-y[0])**2 + (x[1]-y[1])**2
    
    #This is the state of the robot, 0 is awaiting assignment, 1 is assigned
    robot_state = np.zeros(len(robot_positions))
    chunk_to_robot = np.zeros([len(robot_positions),len(printable_chunk_robot_positions)])
    
    #Calculate distance between all robots and robot printing positions
    for i in range(0, len(robot_positions)):
        # print("Robot " + str(i))
        for j in range(0,len(printable_chunk_robot_positions)):
            distance = sqdist(robot_positions[i],printable_chunk_robot_positions[j])
            chunk_to_robot[i][j] =  distance
            
            # print(chunk_to_robot[i][j])
    
    #Sequentially determine which n chunks are the closest to n robots
    printing_chunks = []
    robot_goal_positions = [[]]*len(robot_positions)
    while np.sum(robot_state) != min(len(printable_chunk_robot_positions),len(robot_state)):
        min_val = np.min(chunk_to_robot)
        index_count = np.argmin(chunk_to_robot)
        index = [int(math.floor(index_count/len(printable_chunks))),int(index_count%len(printable_chunks))]
        robot_state[index[0]] = 1
        printing_chunks.append(printable_chunks[index[1]])
        # print("Chunk to robot: " + str(chunk_to_robot))
        robot_schedules[index[0]] = robot_schedules[index[0]] + [printable_chunks[index[1]]]
        
        #Set assigned chunk to arbitrarily high distance to prevent duplicate assignment
        for i in range(0, len(printable_chunk_robot_positions)):
            chunk_to_robot[index[0]][i] = 1000
        
        # print("Chunk to robot: " + str(chunk_to_robot))
        for j in range(0,len(robot_positions)):
            chunk_to_robot[j][index[1]] = 1000
            
        robot_goal_positions[index[0]] = printable_chunk_robot_positions[index[1]]
        # print("Robot State: " +str(robot_state))
    
    #set empty goal positions to same
    for i in range(0,len(robot_state)):
        if robot_state[i] == 0:
            robot_goal_positions[i] = robot_positions[i]

    printing_chunks = sorted(printing_chunks)
    
    #printing_chunks should be a list of indexes of which chunks to print
    return(robot_goal_positions, printing_chunks, robot_schedules)

## Work in progress
def remove_dependencies(chunk_dependencies, printing_chunks):
    for dependencies in chunk_dependencies:
        for chunk in printing_chunks:
            if chunk in dependencies:
                dependencies.remove(chunk)
    return(chunk_dependencies)

#To load the configuration from the floor maker
def load_scenario(fd):
    with open(fd, 'r') as f:
        global GRID_SIZE, ROBOT_RADIUS, RECT_OBSTACLES, START, GOAL
        data = yaml.load(f, Loader=yaml.FullLoader)
        GRID_SIZE = data['GRID_SIZE']
        ROBOT_RADIUS = data['ROBOT_RADIUS']
        RECT_OBSTACLES = data['RECT_OBSTACLES']
        START = data['START']
        GOAL = data['GOAL']
        
def vertices_to_obsts(obsts):
    def drawRect(v0, v1):
        o = []
        base = abs(v0[0] - v1[0])
        side = abs(v0[1] - v1[1])
        for xx in range(0, base, 30):
            o.append((v0[0] + xx, v0[1]))
            o.append((v0[0] + xx, v0[1] + side - 1))
        o.append((v0[0] + base, v0[1]))
        o.append((v0[0] + base, v0[1] + side - 1))
        for yy in range(0, side, 30):
            o.append((v0[0], v0[1] + yy))
            o.append((v0[0] + base - 1, v0[1] + yy))
        o.append((v0[0], v0[1] + side))
        o.append((v0[0] + base - 1, v0[1] + side))
        return o
    static_obstacles = []
    for vs in obsts.values():
        static_obstacles.extend(drawRect(vs[0], vs[1]))
    return static_obstacles


#For testing of this method
def main():
    robot_starting_positions = [[0,0],[1,0],[2,0],[3,0]]
    
    
    floor_size = [8,3]
    
    robot_schedules = [[]]*len(robot_starting_positions)
    
    total_print_time = 0
    obstacles = []   
    
    iterations = 0
    
    while min(chunk_configuration().print_state) <= 0:
    # for iterations in range(0,6):
        print("Iteration: " +str(iterations))
        #Find all printable chunks
        printable_chunks = independent_chunks(chunk_global_info)
        # print("Printable chunks" + str(printable_chunks))
        
        #Find the robot position for all printable chunks
        printable_chunk_robot_positions = robot_chunk_positions(printable_chunks)
        
        (robot_goal_positions, printing_chunks, robot_schedules) =  min_cost(robot_starting_positions, printable_chunk_robot_positions, printable_chunks, robot_schedules)
        # print(printing_chunks)
        
        #Find longest print time of current chunks
        printing_chunks_time = np.zeros(len(printing_chunks))
        for i in range(0,len(printing_chunks)):
            printing_chunks_time[i] = chunk_global_info().chunk_print_time[printing_chunks[i]]
        print_time = max(printing_chunks_time)
        
        #make floor for this configuration
        grid_size_multiplier = floor_maker.make_floor(floor_size[0], floor_size[1], robot_starting_positions, robot_goal_positions, obstacles)
    
        # Load Scenario present in floor
        load_scenario("AMBOTS_floor.yaml")
        
        #Create correct format for static_obstacles
        static_obstacles = vertices_to_obsts(RECT_OBSTACLES)
        
        #call cbs-mapf
        planner_object = planner.Planner(GRID_SIZE, ROBOT_RADIUS, static_obstacles)
        path = planner_object.plan(START, GOAL, debug=False)
        
        #rework the path to avoid diagonals and calculate path distance
        (robot_path_lengths,robot_paths,robot_visualize_paths) = path_scrubber.scrub_paths(path)
        
        #calculate movement time
        robot_move_time = (robot_path_lengths/(grid_size_multiplier/GRID_SIZE))*tuning_variables.robot_speed
        longest_move_time = max(robot_move_time)
        # longest_move_time = 8
        
        #mark printed chunks as printed
        for i in range(0,len(printing_chunks)):
            chunk_configuration().print_state[printing_chunks[i]] = 1
            obstacles.append(chunk_configuration().chunk_positions[printing_chunks[i]])
        # print(chunk_configuration().print_state)
                     
        #remove dependencies of printed chunks
        chunk_configuration().chunk_dependencies = remove_dependencies(chunk_configuration().chunk_dependencies, printing_chunks)
        # print(chunk_configuration.chunk_dependencies)
        
        #set new robot starting positions
        robot_starting_positions = robot_goal_positions
        # print(robot_starting_positions)
        
        total_print_time = total_print_time + longest_move_time + print_time
        
        
        iterations = iterations + 1
        #end loop
    print("Schedule: " + str(robot_schedules))
    print("Total Print Time: " + str(total_print_time))
        
    output_folder = 
    
    # return(path, robot_schedules)
    return(total_print_time)
    
    

if __name__ == '__main__':
    print_time = main()

 ## for randomchunk generation
"""
#the number of chunks
chunk_number = 7
#the printing time (minutes) it takes for each chunk
chunk_print_time = np.round(np.random.rand(1,chunk_number)*10)
#The lowest level chunk upon which each chunk is dependent, can be empty
chunk_dependencies = [[]]*chunk_number
#This is so that it ignores the first 20 percent as having no dependencies
for i in range(int(chunk_number*.2),chunk_number):
    chunk_dependencies[i] = [(int(np.round(np.random.rand()*chunk_number)))]
    
#direction the chunk needs to be printed from 0 = N, 1 = E, etc
print_direction = np.zeros(chunk_number)
for i in range(0, chunk_number):
    print_direction[i] = int(np.floor(np.random.rand()*4))
"""



