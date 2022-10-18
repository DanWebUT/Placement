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
    # robot_speed = .2
    robot_speed = 1

def chunk_print_direction(job_directions, chunk_job):
    chunk_number = len(chunk_job)
    print_direction = np.zeros(chunk_number)
    for i in range(0,chunk_number):
        print_direction[i] = job_directions[chunk_job[i][0]]
    return(print_direction)

def independent_chunks(chunk_number, chunk_dependencies, print_state):
    #find which chunks have no dependencies
    
    printable_chunks = []
    for i  in range(0,chunk_number):
        if chunk_dependencies[i] == [] and print_state[i] == 0:
            printable_chunks.append(i)
            
    
    return(printable_chunks)

def robot_chunk_positions(printable_chunks, chunk_positions, print_direction):
    printable_chunk_robot_positions = []
    for i in range (0,len(printable_chunks)):
        if print_direction[printable_chunks[i]] == 0:
            x_pos = chunk_positions[printable_chunks[i]][0]
            y_pos = chunk_positions[printable_chunks[i]][1]-1
        if print_direction[printable_chunks[i]] == 1:
            x_pos = chunk_positions[printable_chunks[i]][0]+1
            y_pos = chunk_positions[printable_chunks[i]][1]
        if print_direction[printable_chunks[i]] == 2:
            x_pos = chunk_positions[printable_chunks[i]][0]
            y_pos = chunk_positions[printable_chunks[i]][1]+1
        if print_direction[printable_chunks[i]] == 3:
            x_pos = chunk_positions[printable_chunks[i]][0]-1
            y_pos = chunk_positions[printable_chunks[i]][1]
            
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
        data = yaml.load(f, Loader=yaml.FullLoader)
        GRID_SIZE = data['GRID_SIZE']
        ROBOT_RADIUS = data['ROBOT_RADIUS']
        RECT_OBSTACLES = data['RECT_OBSTACLES']
        START = data['START']
        GOAL = data['GOAL']
        return(GRID_SIZE, ROBOT_RADIUS, RECT_OBSTACLES, START, GOAL)
        
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
def schedule(robot_starting_positions, floor_size, chunk_dependencies, chunk_job, chunk_print_time, chunk_positions, print_direction):
    robot_schedules = [[]]*len(robot_starting_positions)
    chunk_number = len(chunk_job)    
    print_state = np.zeros(chunk_number)
    robot_state = np.zeros(len(robot_starting_positions))
    time_until_next_action = np.zeros(len(robot_starting_positions))
    
    total_print_time = 0
    obstacles = []   
    
    iterations = 0
    global_time = 0
    
    while min(print_state) <= 0:
    # for iterations in range(0,6):
        # print("Iteration: " +str(iterations))
        
        #Find all printable chunks
        printable_chunks = independent_chunks(chunk_number, chunk_dependencies, print_state)
        print("Printable chunks" + str(printable_chunks))
        
        #check if there is an available robot
        if min(robot_state) == 0:
            #check if there is a printable chunk
            if printable_chunks != []:
                #Find the robot position for all printable chunks
                printable_chunk_robot_positions = robot_chunk_positions(printable_chunks, chunk_positions, print_direction)
        
                (robot_goal_positions, printing_chunks, robot_schedules) =  min_cost(robot_starting_positions, printable_chunk_robot_positions, printable_chunks, robot_schedules)
                print("Printing Chunks: " + str(printing_chunks))
        
                
                #calculate robot path and path time
                #make floor for this configuration
                grid_size_multiplier = floor_maker.make_floor(floor_size[0], floor_size[1], robot_starting_positions, robot_goal_positions, obstacles)
            
                # Load Scenario present in floor
                (GRID_SIZE, ROBOT_RADIUS, RECT_OBSTACLES, START, GOAL) = load_scenario("AMBOTS_floor.yaml")
                
                #Create correct format for static_obstacles
                static_obstacles = vertices_to_obsts(RECT_OBSTACLES)
                
                #call cbs-mapf
                planner_object = planner.Planner(GRID_SIZE, ROBOT_RADIUS, static_obstacles)
                path = planner_object.plan(START, GOAL, debug=False)
                
                try: 
                    path_error = False
                    #rework the path to avoid diagonals and calculate path distance
                    (robot_path_lengths,robot_paths,robot_visualize_paths) = path_scrubber.scrub_paths(path)
                    
                    #WIP
                    #calculate movement time for each robot
                    robot_move_time = (robot_path_lengths/(grid_size_multiplier/GRID_SIZE))*tuning_variables.robot_speed
        
        
        
                    #WIP
                    #Find shortest print time and which robot is printing that chunk
                    printing_chunks_time = np.zeros(len(printing_chunks))
                    for i in range(0,len(printing_chunks)):
                        printing_chunks_time[i] = chunk_print_time[printing_chunks[i]]
                    print_time = max(printing_chunks_time)
        
                    
                    
        
                    #WIP
                    #mark only printed chunks as printed
                    for i in range(0,len(printing_chunks)):
                        print_state[printing_chunks[i]] = 1
                        obstacles.append(chunk_positions[printing_chunks[i]])
                    # print(chunk_configuration().print_state)
                                 
                    #remove dependencies of printed chunks
                    chunk_dependencies = remove_dependencies(chunk_dependencies, printing_chunks)
                    # print(chunk_configuration.chunk_dependencies)
                    
                    #set new robot starting positions
                    robot_starting_positions = robot_goal_positions
                    # print(robot_starting_positions)
                    
                    iterations = iterations + 1
                    
        
                except ValueError:
                    path_error = True
                    print_state = [1]
                    total_print_time = 1000
        
        
                
        
            longest_move_time = max(robot_move_time)
            
            #WIP
            #Check if another robot can start moving during current move phase
            
            #If yes, stop the move at the position where the next move would start and continue
        
            #add movement time to print time
            total_print_time = total_print_time + longest_move_time + print_time
            
            
        
            #end loop
    # print("Schedule: " + str(robot_schedules))
    # print("Total Print Time: " + str(total_print_time))
        
    # output_folder = 
    
    # return(path, robot_schedules)
    return(total_print_time, path_error)
    
    

if __name__ == '__main__':
    chunk_dependencies = [[],[0],[],[2],[],[4],[],[6],\
                          [0],[1,8],[2],[3,10],[4],[5,12],[6],[7,14],\
                          [8],[9,16],[10],[11,18],[12],[13,20],[14],[15,22]]
    chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]
    chunk_print_time = [89., 82., 89., 84., 85., 80., 83., 87., 86., 81., 82., 87., 84., 64., 67., 78., \
            84., 94., 87., 49., 86., 89., 86., 83.]
    robot_starting_positions = [[0,0],[1,0],[2,0],[3,0]]
    floor_size = [8,6]
    # chunk_positions = [[0,5],[1,5],[13,1],[14,1],[5,2],[5,1],[9,4],[9,5],\
    #                       [0,4],[1,4],[13,2],[14,2],[6,2],[6,1],[10,4],[10,5],\
    #                       [0,3],[1,3],[13,3],[14,3],[7,2],[7,1],[11,4],[11,5]]
    
    chunk_positions = [[14, 2],[14, 3], [7, 0],[8, 0],[7, 8],[7, 9],[13, 7],[14, 7], \
                       [13, 2],     [13, 3],     [7, 1],     [8, 1],     [6, 8],     [6, 9],     [13, 8],     [14, 8],\
                        [12, 2],     [12, 3],     [7, 2],     [8, 2],     [5, 8],     [5, 9],     [13, 9],     [14, 9]]
        
    
    chunk_number = len(chunk_job)
    # job_directions = [0,2,1,1]   
    job_directions = [3, 2, 3, 2]   
    print_direction = chunk_print_direction(job_directions, chunk_job)
                      
    (total_print_time, path_error) = schedule(robot_starting_positions, floor_size, chunk_dependencies, chunk_job, chunk_print_time, chunk_positions, print_direction)
    print("Total Print Time: " + str(total_print_time))
    
    
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



