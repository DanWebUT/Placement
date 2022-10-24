#This program is used to track the progress of the printing

"""
This program will track the steps of the printing process (printing and move
phase) and call the floor maker and cbs algorithm at each step of the process
"""
import numpy as np
import floor_maker
# import placement_visualizer
# import visualizer
import yaml
import math
# import time
import path_analyzer
import copy
import os
import sys

current_path = os.getcwd()
(PythonCBS, visualization) = os.path.split(current_path)
(Placement, PythonCBS) = os.path.split(PythonCBS)
filepath = os.path.join(Placement, 'PythonCBS\cbs_mapf')

sys.path.insert(0, filepath)

from planner import Planner


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
    

def min_cost(robot_starting_positions, printable_chunk_robot_positions, printable_chunks, robot_schedules, global_robot_states):
    # Greedy algorithm to assign chunks to all the agents
    # Just needs to output the robot positions
    sqdist = lambda x, y: (x[0]-y[0])**2 + (x[1]-y[1])**2
    
    moveable_robots = []
    moving_robots = []
    #figure out which can are moving
    for i in range(0, len(global_robot_states)):
        if global_robot_states[i] == 0:
            moveable_robots.append(i)
    
    #This is the state of the robot, 0 is awaiting assignment, 1 is assigned
    robot_state = np.zeros(len(robot_starting_positions))
    chunk_to_robot = np.zeros([len(robot_starting_positions),len(printable_chunk_robot_positions)])
    
    #Calculate distance between all robots and robot printing positions
    for i in range(0, len(robot_starting_positions)):
        # print("Robot " + str(i))
        for j in range(0,len(printable_chunk_robot_positions)):
            distance = sqdist(robot_starting_positions[i],printable_chunk_robot_positions[j])
            chunk_to_robot[i][j] =  distance
            
            # print(chunk_to_robot[i][j])
    
    #Sequentially determine which n chunks are the closest to n robots
    printing_chunks = []
    robot_goal_positions = []
    new_robot_starting_positions = []
    while np.sum(robot_state) != min(len(printable_chunk_robot_positions),len(robot_state)):
        index_count = np.argmin(chunk_to_robot)
        index = [int(math.floor(index_count/len(printable_chunks))),int(index_count%len(printable_chunks))]
        robot_state[index[0]] = 1
        moving_robots.append(moveable_robots[index[0]])
        printing_chunks.append(printable_chunks[index[1]])
        # print("Chunk to robot: " + str(chunk_to_robot))
        # robot_schedules[moveable_robots[index[0]]] = robot_schedules[moveable_robots[index[0]]] + [printable_chunks[index[1]]]
        
        #Set assigned chunk to arbitrarily high distance to prevent duplicate assignment
        for i in range(0, len(printable_chunk_robot_positions)):
            chunk_to_robot[index[0]][i] = 1000
        
        # print("Chunk to robot: " + str(chunk_to_robot))
        for j in range(0,len(robot_starting_positions)):
            chunk_to_robot[j][index[1]] = 1000
            
        robot_goal_positions.append(printable_chunk_robot_positions[index[1]])
        new_robot_starting_positions.append(robot_starting_positions[index[0]])
        # print("Robot State: " +str(robot_state))
        
        if len(moving_robots) == len(printable_chunks):
            break
    
    moving_robots.sort()
    
    # #set empty goal positions to same
    # for i in range(0,len(robot_state)):
    #     if robot_state[i] == 0:
    #         robot_goal_positions.append(robot_starting_positions[i])
    
    #printing_chunks should be a list of indexes of which chunks to print
    return(new_robot_starting_positions, robot_goal_positions, printing_chunks, moving_robots)

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
    robot_schedules = [[]]
    for i in range(0,len(robot_starting_positions)-1):
        robot_schedules.append([])
    chunk_number = len(chunk_job)    
    chunk_print_states = np.zeros(chunk_number)
    robot_states = np.zeros(len(robot_starting_positions),int)
    time_current_action = np.zeros(len(robot_starting_positions))
    robot_positions = copy.deepcopy(robot_starting_positions)
    robot_last_chunk_printed = np.zeros(len(robot_positions),int)
    
    total_print_time = 0
    chunk_obstacles = []
    robot_obstacles = []
    obstacles = chunk_obstacles+robot_obstacles
    
    elapsed_time = 0
    
    iteration_count = 0
    
    while min(chunk_print_states) <= 1 and iteration_count <=50:
        iteration_count += 1
        #Find all printable chunks
        printable_chunks = independent_chunks(chunk_number, chunk_dependencies, chunk_print_states)
        
        #FOR TESTING
        # print("Printable chunks" + str(printable_chunks))
        
        # if printable_chunks == [16]:
            # print("Debug Here")
        
        #check if there is an available robot
        if min(robot_states) == 0:
            #check if there is a printable chunk
            if printable_chunks != []:
                #Find the robot position for all printable chunks
                printable_chunk_robot_positions = robot_chunk_positions(printable_chunks, chunk_positions, print_direction)
                
                #Update this method to take in all robot states and assigne the ones that are waiting
                (robot_starting_positions, robot_goal_positions, printing_chunks, moving_robots) =  min_cost(robot_starting_positions, printable_chunk_robot_positions, printable_chunks, robot_schedules, robot_states)
                
                #set non-moving robots as obstacles
                if len(robot_states) - sum(robot_states) > len(moving_robots):
                    #calculate global elapsed time based on second smallest
                    time_no_zero = copy.deepcopy(time_current_action)
                    for time_var in range(0, len(time_no_zero)):
                        if time_no_zero[time_var] == 0:
                            time_no_zero[time_var] = max(time_no_zero)
                
                    time_skip = min(time_no_zero)
                    robot_obstacles = [] 
                    for robot in range(0, len(robot_states)):
                        if ((np.isin(moving_robots,robot))[0]) == False:
                            robot_obstacles.append(robot_positions[robot])
                            if time_current_action[robot] == 0:
                                robot_states[robot] = 2
                                time_current_action[robot] += time_skip
                            
                    obstacles = chunk_obstacles+robot_obstacles
                    
                    
               
                
                #FOR TESTING
                # print("Printing Chunks: " + str(printing_chunks))
                
                #update robots to move state
                for count, robot in enumerate(moving_robots):
                    robot_states[robot] = 1
                        
                #mark all printing chunks as in progress
                for chunk in printing_chunks:
                    chunk_print_states[chunk] = 1
                
                #calculate robot path and path time
                #make floor for this configuration
                grid_size_multiplier = floor_maker.make_floor(floor_size[0], floor_size[1], robot_starting_positions, robot_goal_positions, obstacles)
            
                # Load Scenario present in floor
                (GRID_SIZE, ROBOT_RADIUS, RECT_OBSTACLES, START, GOAL) = load_scenario("AMBOTS_floor.yaml")
                
                #Create correct format for static_obstacles
                static_obstacles = vertices_to_obsts(RECT_OBSTACLES)
                
                #call cbs-mapf
                planner_object = Planner(GRID_SIZE, ROBOT_RADIUS, static_obstacles)
                paths = planner_object.plan(START, GOAL, debug=False)
                
                try: 
                    path_error = False
                    
                    #FOR TESTING
                    # print("Moving Robot: "+ str(moving_robots) )
                    # print("Robot Positions: " + str(robot_positions))
                    
                    #calculate path length and update robot positions and schedules
                    (robot_path_lengths,robot_schedules,robot_positions,robot_last_chunk_printed) = path_analyzer.analyze_paths(paths, robot_positions, robot_goal_positions, printing_chunks, moving_robots, robot_schedules, robot_last_chunk_printed)
                    
                    #FOR TESTING
                    # print("Robot Positions after move: " + str(robot_positions))
                    # print("Paths: \n" +str(paths))
                    
                    
                    
                    #calculate movement time for each robot
                    robot_move_time = ((robot_path_lengths))*tuning_variables.robot_speed
                    
                    #visualize each step
                    # visualizer.visualizer()
                    
                    #FOR TESTING
                    # print("Robot Move Time " + str(robot_move_time))
                    
                    #add movement time to current action queue
                    time_current_action += robot_move_time
                    
                    #FOR TESTING
                    # print("Time Current Action: " +str(time_current_action))
                    
                    #Find print time for all robots and add to time queue
                    printing_chunks_time = np.zeros(len(robot_positions))
                    for i in range(0,len(printing_chunks)):
                        robot = np.where(robot_last_chunk_printed == printing_chunks[i])[0][0]
                        printing_chunks_time[robot] = chunk_print_time[printing_chunks[i]]
                    
                    # print("Chunk Print Times "+ str(printing_chunks_time))    
                    
                    #add print time to current action queue
                    time_current_action += printing_chunks_time
                        
                    #calculate global elapsed time based on shortest next action time
                    elapsed_time += (min(time_current_action))
                    time_current_action -= (min(time_current_action))
                    
                    # print("Elapsed Time: " + str(elapsed_time))
                    # print("Time Current Action: " +str(time_current_action))
                    
                    #set robot states of finished robots back to waiting
                    num_chunks_printed_iteration = 0
                    robots_finished = []
                    for i in range(0,len(robot_states)):
                        if time_current_action[i] == 0 and robot_states[i] == 1:
                            robot_states[i] = 0
                            num_chunks_printed_iteration += 1
                            robots_finished.append(i)
                        elif robot_states[i] == 2:
                            robot_states[i] = 0
                    
                    #mark the chunk that finished printing as printed
                    printed_chunks = []
                    for i in range(0,num_chunks_printed_iteration):
                        printed_chunks.append(robot_schedules[robots_finished[i]][-1])
                    
                    #mark only printed chunks as printed
                    for i in range(0,len(printed_chunks)):
                        chunk_print_states[printed_chunks[i]] = 2
                        
                        
                    #mark all chunks in progress as obstacles
                    for i in range(0,len(printing_chunks)):
                        chunk_obstacles.append(chunk_positions[printing_chunks[i]])
                        
                    #remove dependencies of printed chunks
                    chunk_dependencies = remove_dependencies(chunk_dependencies, printed_chunks)
                    # print(chunk_configuration.chunk_dependencies)
                    
                    #set new robot starting positions and set working robots as obstacles
                    robot_starting_positions = []
                    robot_obstacles = []
                    for i in range(0,len(robot_states)):
                        if robot_states[i] == 0:
                            robot_starting_positions.append(robot_positions[i])
                        else:
                            robot_obstacles.append(robot_positions[i])
                            
                    obstacles = chunk_obstacles+robot_obstacles
                
                except ValueError:
                    # print("Path Error")
                    path_error = True
                    chunk_print_states = [2]
                    elapsed_time = 1000
                        
                except IndexError:
                    path_error = True
                    chunk_print_states = [2]
                    elapsed_time = 1000
                
            else:
                #no available chunks to be printed
                if min(chunk_print_states) >0:
                    time_remaining = max(time_current_action)
                    elapsed_time += time_remaining
                    time_current_action -= time_remaining
                    
                    #find which chunks are still under construction
                    printing_chunks = []
                    for chunk in range(0,len(chunk_print_states)):
                        if chunk_print_states[chunk] == 1:
                            printing_chunks.append(chunk)
                            chunk_print_states[chunk] = 2
                    
                    
                else:
                    #FOR TESTING
                    # print("Debug print states")
                    
                    #there are still unprintable chunks
                    #identify stuck robots
                    stuck_robots = []
                    for robot in range(0,len(robot_states)):
                        if robot_states[robot] == 0:
                            position = copy.deepcopy(robot_positions[robot])
                            #make sure robot is actually stuck
                            if np.any(np.all(np.array(position)==np.array(chunk_positions), axis = 1)) or np.any(np.all(np.array(position)==np.array(obstacles), axis = 1)):
                                stuck_robots.append(robot)
                            
                    #run path planning for stuck robots
                    if stuck_robots != []:
                        #find the direction of the last chunk printed
                        dir_last_chunk = []
                        for i in range(0,len(stuck_robots)):
                            dir_last_chunk.append(print_direction[robot_last_chunk_printed[stuck_robots[i]]])
                        
                        #find suitable place to move the robot to
                        robot_goal_positions = []
                        new_robot_starting_positions = []
                        for count, robot in enumerate(stuck_robots):
                            position = copy.deepcopy(robot_positions[robot])
                            while np.any(np.all(np.array(position)==np.array(chunk_positions), axis = 1)) or np.any(np.all(np.array(position)==np.array(obstacles), axis = 1)):
                                if dir_last_chunk[count] == 0:
                                    position[1] += 2
                                elif dir_last_chunk[count] == 1:
                                    position[0] += 2
                                elif dir_last_chunk[count] == 2:
                                    position[1] -= 2
                                elif dir_last_chunk[count] == 3:
                                       position[0] -= 2
                            
                            robot_goal_positions.append(position)
                            new_robot_starting_positions.append(robot_positions[robot])
                            
                        #calculate robot path and path time
                        #make floor for this configuration
                        grid_size_multiplier = floor_maker.make_floor(floor_size[0], floor_size[1], new_robot_starting_positions, robot_goal_positions, obstacles)
                    
                        # Load Scenario present in floor
                        (GRID_SIZE, ROBOT_RADIUS, RECT_OBSTACLES, START, GOAL) = load_scenario("AMBOTS_floor.yaml")
                        
                        #Create correct format for static_obstacles
                        static_obstacles = vertices_to_obsts(RECT_OBSTACLES)
                        
                        #call cbs-mapf
                        planner_object = Planner(GRID_SIZE, ROBOT_RADIUS, static_obstacles)
                        paths = planner_object.plan(START, GOAL, debug=False)
                        
                        #calculate path length and update robot positions and schedules
                        (robot_path_lengths,robot_schedules,robot_positions,robot_last_chunk_printed) = path_analyzer.analyze_paths(paths, robot_positions, robot_goal_positions, printing_chunks, moving_robots, robot_schedules, robot_last_chunk_printed)
                        
                        #calculate movement time for each robot
                        robot_move_time = ((robot_path_lengths))*tuning_variables.robot_speed
                        
                        #visualize each step
                        # visualizer.visualizer()
                        
                        #FOR TESTING
                        # print("Robot Move Time " + str(robot_move_time))
                        
                        #add movement time to current action queue
                        time_current_action += robot_move_time
                        
                        #calculate global elapsed time based on shortest next action time
                        elapsed_time += (min(time_current_action))
                        time_current_action -= (min(time_current_action))
                        
                        #set robot states of finished robots back to waiting
                        robots_finished = []
                        for i in range(0,len(robot_states)):
                            if time_current_action[i] == 0:
                                robot_states[i] = 0
                                robots_finished.append(i)
                            else:
                                robot_states[i] = 1
                        
                        #set new robot starting positions and set working robots as obstacles
                        robot_starting_positions = []
                        robot_obstacles = []
                        for i in range(0,len(robot_states)):
                            if robot_states[i] == 0:
                                robot_starting_positions.append(robot_positions[i])
                            else:
                                robot_obstacles.append(robot_positions[i])
                                
                        obstacles = chunk_obstacles+robot_obstacles
                        
                    else:
                        #elapse time to finish a print if no robot stuck
                        #calculate global elapsed time based on second smallest
                        time_no_zero = copy.deepcopy(time_current_action)
                        for time_var in range(0, len(time_no_zero)):
                            if time_no_zero[time_var] == 0:
                                time_no_zero[time_var] = max(time_no_zero)
                    
                        time_skip = min(time_no_zero)
                    
                        elapsed_time += (min(time_no_zero))
                        time_current_action -= (time_skip)
                            
                        # print("Elapsed Time: " + str(elapsed_time))
                        # print("Time Current Action: " +str(time_current_action))
                        
                        #set robot states of finished robots back to waiting
                        num_chunks_printed_iteration = 0
                        robots_finished = []
                        for i in range(0,len(robot_states)):
                            if time_current_action[i] == 0 and robot_states[i] == 1:
                                robot_states[i] = 0
                                num_chunks_printed_iteration += 1
                                robots_finished.append(i)
                            elif robot_states[i] == 2:
                                robot_states[i] = 0
                        
                        #mark the chunk that finished printing as printed
                        printed_chunks = []
                        for i in range(0,num_chunks_printed_iteration):
                            printed_chunks.append(robot_schedules[robots_finished[i]][-1])
                        
                        #mark only printed chunks as printed
                        for i in range(0,len(printed_chunks)):
                            chunk_print_states[printed_chunks[i]] = 2
                            
                        #set all times less than 0 to zero
                        for time in range(0, len(time_current_action)):
                            if time_current_action[time] <0:
                                time_current_action[time] = 0
                        
                        #mark all chunks in progress as obstacles
                        for i in range(0,len(printing_chunks)):
                            chunk_obstacles.append(chunk_positions[printing_chunks[i]])
                            
                        #remove dependencies of printed chunks
                        chunk_dependencies = remove_dependencies(chunk_dependencies, printed_chunks)
                        # print(chunk_configuration.chunk_dependencies)
                        
                        #set new robot starting positions and set working robots as obstacles
                        robot_starting_positions = []
                        robot_obstacles = []
                        for i in range(0,len(robot_states)):
                            if robot_states[i] == 0:
                                robot_starting_positions.append(robot_positions[i])
                            else:
                                robot_obstacles.append(robot_positions[i])
                                
                        obstacles = chunk_obstacles+robot_obstacles
                        
                    
        else:
            #FOR TESTING
            # print("All robots in use")
            
            #calculate global elapsed time based on shortest next action time
            elapsed_time += (min(time_current_action))
            time_current_action -= (min(time_current_action))
            
            # print("Elapsed Time: " + str(elapsed_time))
            # print("Time Current Action: " +str(time_current_action))
            
            #set robot states of finished robots back to waiting
            num_chunks_printed_iteration = 0
            robots_finished = []
            for i in range(0,len(robot_states)):
                if time_current_action[i] == 0:
                    robot_states[i] = 0
                    num_chunks_printed_iteration += 1
                    robots_finished.append(i)
            
            #mark the chunk that finished printing as printed
            printed_chunks = []
            for i in range(0,num_chunks_printed_iteration):
                printed_chunks.append(robot_schedules[robots_finished[i]][-1])
            
            #mark only printed chunks as printed
            for i in range(0,len(printed_chunks)):
                chunk_print_states[printed_chunks[i]] = 2
                
            #mark all chunks in progress as obstacles
            for i in range(0,len(printing_chunks)):
                chunk_obstacles.append(chunk_positions[printing_chunks[i]])
                         
            #remove dependencies of printed chunks
            chunk_dependencies = remove_dependencies(chunk_dependencies, printed_chunks)
            # print(chunk_configuration.chunk_dependencies)
            
            #set new robot starting positions and set working robots as obstacles
            robot_starting_positions = []
            robot_obstacles = []
            for i in range(0,len(robot_states)):
                if robot_states[i] == 0:
                    robot_starting_positions.append(robot_positions[i])
                else:
                    robot_obstacles.append(robot_positions[i])
                    
            obstacles = chunk_obstacles+robot_obstacles
    
    total_print_time = elapsed_time
    
    # print("Total Print Time: "+str(total_print_time))
    
    return(total_print_time, path_error)
    
    

if __name__ == '__main__':
    # chunk_dependencies = [[],[0],[],[2],[],[4],[],[6],\
    #                       [0],[1,8],[2],[3,10],[4],[5,12],[6],[7,14],\
    #                       [8],[9,16],[10],[11,18],[12],[13,20],[14],[15,22]]
    # chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]
    # chunk_positions = [[10, 6], [9, 6], [13, 0], [13, 1], [4, 6], [4, 7], [5, 0], [6, 0], [10, 5], [9, 5], [12, 0], [12, 1], [3, 6], [3, 7], [5, 1], [6, 1], [10, 4], [9, 4], [11, 0], [11, 1], [2, 6], [2, 7], [5, 2], [6, 2]]
    # job_directions = [0, 3, 3, 2] 
    
    #rotated tall box
    chunk_dependencies = [[],[0, 2],[],[],[3, 5],[],[],[6, 8], [],[],[9, 11],[],\
                          [0],[1,12,14],[2],[3],[4,15,17],[5],[6],[7,18,20],[8],[9],[10,21,23],[11]]
        
    chunk_job = [[0],[0],[0],[1],[1],[1],[2],[2],[2],[3],[3],[3],[0],[0],[0],[1],[1],[1],[2],[2],[2],[3],[3],[3]]
    job_directions = [2, 1, 3, 1]
    chunk_positions = [[2, 9], [3, 9], [4, 9], [8, 9], [8, 8], [8, 7], [6, 2], [6, 3], [6, 4], [10, 5], [10, 4], [10, 3], [2, 10], [3, 10], [4, 10], [9, 9], [9, 8], [9, 7], [5, 2], [5, 3], [5, 4], [11, 5], [11, 4], [11, 3]]
    
    chunk_print_time = [89., 82., 89., 84., 85., 80., 83., 87., 86., 81., 82., 87., 84., 64., 67., 78., \
            84., 94., 87., 49., 86., 89., 86., 83.]
    robot_starting_positions = [[0,0],[1,0],[2,0],[3,0]]
    floor_size = [8,6]
    # chunk_positions = [[0,5],[1,5],[13,1],[14,1],[5,2],[5,1],[9,4],[9,5],\
    #                       [0,4],[1,4],[13,2],[14,2],[6,2],[6,1],[10,4],[10,5],\
    #                       [0,3],[1,3],[13,3],[14,3],[7,2],[7,1],[11,4],[11,5]]
    
    # chunk_positions = [[14, 2],[14, 3], [7, 1],[8, 1],[7, 8],[7, 9],[13, 7],[14, 7], \
    #                    [13, 2],     [13, 3],     [7, 2],     [8, 2],     [6, 8],     [6, 9],     [13, 8],     [14, 8],\
    #                     [12, 2],     [12, 3],     [7, 3],     [8, 3],     [5, 8],     [5, 9],     [13, 9],     [14, 9]]
    
       
    
    # placement_visualizer.placement_vis(floor_size, chunk_positions, chunk_job)
        
    chunk_number = len(chunk_job)
    # job_directions = [0,2,1,1]   
    
    print_direction = chunk_print_direction(job_directions, chunk_job)
                      
    (total_print_time, path_error) = schedule(robot_starting_positions, floor_size, chunk_dependencies, chunk_job, chunk_print_time, chunk_positions, print_direction)
    print("Total Print Time: " + str(total_print_time))
    
    


