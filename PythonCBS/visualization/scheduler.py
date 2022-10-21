#This program is used to track the progress of the printing

"""
This program will track the steps of the printing process (printing and move
phase) and call the floor maker and cbs algorithm at each step of the process
"""
import numpy as np
import floor_maker
import placement_visualizer
import visualizer
import yaml
import math
import time
import path_scrubber
import copy

from cbs_mapf import planner


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
    

def min_cost(robot_positions, printable_chunk_robot_positions, printable_chunks, robot_schedules, global_robot_states):
    # Greedy algorithm to assign chunks to all the agents
    # Just needs to output the robot positions
    sqdist = lambda x, y: (x[0]-y[0])**2 + (x[1]-y[1])**2
    
    moving_robots = []
    #figure out which robots are moving
    for i in range(0, len(global_robot_states)):
        if global_robot_states[i] == 0:
            moving_robots.append(i)
    
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
        robot_schedules[moving_robots[index[0]]] = robot_schedules[moving_robots[index[0]]] + [printable_chunks[index[1]]]
        
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
    return(robot_goal_positions, printing_chunks, robot_schedules, moving_robots)

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
    chunk_print_states = np.zeros(chunk_number)
    robot_states = np.zeros(len(robot_starting_positions),int)
    time_current_action = np.zeros(len(robot_starting_positions))
    robot_positions = copy.deepcopy(robot_starting_positions)
    
    total_print_time = 0
    chunk_obstacles = []
    robot_obstacles = []
    obstacles = chunk_obstacles+robot_obstacles
    
    elapsed_time = 0
    
    
    
    while min(chunk_print_states) <= 1:
        
        #Find all printable chunks
        printable_chunks = independent_chunks(chunk_number, chunk_dependencies, chunk_print_states)
        
        print("Printable chunks" + str(printable_chunks))
        
        #check if there is an available robot
        if min(robot_states) == 0:
            #check if there is a printable chunk
            if printable_chunks != []:
                #Find the robot position for all printable chunks
                printable_chunk_robot_positions = robot_chunk_positions(printable_chunks, chunk_positions, print_direction)
                
                #Update this method to take in all robot states and assigne the ones that are waiting
                (robot_goal_positions, printing_chunks, robot_schedules, moving_robots) =  min_cost(robot_starting_positions, printable_chunk_robot_positions, printable_chunks, robot_schedules, robot_states)
                
                print("Printing Chunks: " + str(printing_chunks))
                
                if printing_chunks == [15]: 
                    print("Debug here")
                
                #update robots to move state
                for count, robot in enumerate(moving_robots):
                    robot_states[robot] = 1
                    robot_positions[robot] = robot_goal_positions[count]
                        
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
                planner_object = planner.Planner(GRID_SIZE, ROBOT_RADIUS, static_obstacles)
                path = planner_object.plan(START, GOAL, debug=False)
                
                try: 
                    path_error = False
                    #rework the path to avoid diagonals and calculate path distance
                    (robot_path_lengths,robot_paths,robot_visualize_paths) = path_scrubber.scrub_paths(path)
                    
                    #visualize each step
                    visualizer.visualizer()
                    
                    #calculate movement time for each robot
                    robot_move_time = ((robot_path_lengths)/(2))*tuning_variables.robot_speed
                    
                    # print("Robot Move Time " + str(robot_move_time))
                    
                    #add movement time to current action queue
                    for count, robot in enumerate(moving_robots):
                        time_current_action[robot] += (robot_move_time[count])
                    
                    # print("Time Current Action: " +str(time_current_action))
                    
                    #Find print time for all robots and add to time queue
                    printing_chunks_time = np.zeros(len(printing_chunks))
                    for i in range(0,len(printing_chunks)):
                        printing_chunks_time[i] = chunk_print_time[printing_chunks[i]]
                    
                    # print("Chunk Print Times "+ str(printing_chunks_time))    
                    
                    #add print time to current action queue
                    for count, robot in enumerate(moving_robots):
                        time_current_action[robot] += (printing_chunks_time[count])
                        
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
                    
                except ValueError:
                    #robot is probably stuck somewhere, see if we can get it out
                    """
                    This part is super frustrating because it seems the path planning algorithm doesn't try any paths
                    that ever move away from the goal position. Maybe the low level path planning needs to be adjusted
                    
                    """
                    try:
                        print_direction_chunk = np.zeros(len(printing_chunks))
                        add_time = np.zeros(len(printing_chunks))
                        new_robot_starting_positions = copy.deepcopy(robot_starting_positions)
                        for count, chunk in enumerate(printing_chunks):
                            #Need print direction of chunk robot has just finished printing
                            print_direction_chunk[count] = print_direction[robot_schedules[moving_robots[count]][-2]]
                            
                            #move the starting position two back from the print location
                            if print_direction_chunk == 0:
                                new_robot_starting_positions[count][1] = new_robot_starting_positions[count][1] - 2
                                if new_robot_starting_positions[count][1] < 0 :
                                    add_time[count] = 2*tuning_variables.robot_speed
                                    new_robot_starting_positions[count][1] = new_robot_starting_positions[count][1] + 1
                                    if [(new_robot_starting_positions[count][0]+1),new_robot_starting_positions[count][1]] in obstacles:
                                        new_robot_starting_positions[count][0] = new_robot_starting_positions[count][0] - 1
                                    else:
                                        new_robot_starting_positions[count][0] = new_robot_starting_positions[count][0] + 1
                            elif print_direction_chunk == 1:
                                new_robot_starting_positions[count][0] = new_robot_starting_positions[count][0] + 2
                                if new_robot_starting_positions[count][0] > floor_size[0]*2-1:
                                    add_time[count] = 2*tuning_variables.robot_speed
                                    new_robot_starting_positions[count][0] = new_robot_starting_positions[count][0] - 1
                                    if [(new_robot_starting_positions[count][0]),(new_robot_starting_positions[count][1]+1)] in obstacles:
                                        new_robot_starting_positions[count][1] = new_robot_starting_positions[count][1] - 1
                                    else:
                                        new_robot_starting_positions[count][1] = new_robot_starting_positions[count][1] + 1
                            elif print_direction_chunk == 2:
                                new_robot_starting_positions[count][1] = new_robot_starting_positions[count][1] + 2
                                if new_robot_starting_positions[count][1] > floor_size[1]*2-1:
                                    add_time[count] = 2*tuning_variables.robot_speed
                                    new_robot_starting_positions[count][1] = new_robot_starting_positions[count][1] - 1
                                    if [(new_robot_starting_positions[count][0]+1),new_robot_starting_positions[count][1]] in obstacles:
                                        new_robot_starting_positions[count][0] = new_robot_starting_positions[count][0] - 1
                                    else:
                                        new_robot_starting_positions[count][0] = new_robot_starting_positions[count][0] + 1
                            else:
                                new_robot_starting_positions[count][0] = new_robot_starting_positions[count][0] - 2
                                if new_robot_starting_positions[count][0] < 0:
                                    add_time[count] = 2*tuning_variables.robot_speed
                                    new_robot_starting_positions[count][0] = new_robot_starting_positions[count][0] + 1
                                    if [(new_robot_starting_positions[count][0]),(new_robot_starting_positions[count][1]+1)] in obstacles:
                                        new_robot_starting_positions[count][1] = new_robot_starting_positions[count][1] - 1
                                    else:
                                        new_robot_starting_positions[count][1] = new_robot_starting_positions[count][1] + 1
                        
                        #and then same as before, calculate robot time and do all the stuff
                        
                        #calculate robot path and path time
                        #make floor for this configuration
                        grid_size_multiplier = floor_maker.make_floor(floor_size[0], floor_size[1], new_robot_starting_positions, robot_goal_positions, obstacles)
                    
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
                            
                            #visualize each step
                            visualizer.visualizer()
                            
                            #calculate movement time for each robot
                            robot_move_time = ((robot_path_lengths)/(2))*tuning_variables.robot_speed + add_time
                            
                            # print("Robot Move Time " + str(robot_move_time))
                            
                            #add movement time to current action queue
                            for count, robot in enumerate(moving_robots):
                                time_current_action[robot] += (robot_move_time[count])
                            
                            # print("Time Current Action: " +str(time_current_action))
                            
                            #Find print time for all robots and add to time queue
                            printing_chunks_time = np.zeros(len(printing_chunks))
                            for i in range(0,len(printing_chunks)):
                                printing_chunks_time[i] = chunk_print_time[printing_chunks[i]]
                            
                            # print("Chunk Print Times "+ str(printing_chunks_time))    
                            
                            #add print time to current action queue
                            for count, robot in enumerate(moving_robots):
                                time_current_action[robot] += (printing_chunks_time[count])
                                
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
                        
                        except ValueError:
                            #move the starting position Diagonal next to the first chunk from the print location
                            print_direction_chunk = np.zeros(len(printing_chunks))
                            add_time = np.zeros(len(printing_chunks))
                            current_chunk_jobs = np.zeros(len(printing_chunks))
                            
                            #find job initial_chunks
                            initial_chunks = []
                            for job in range(0,len(job_directions)):
                                initial_chunks.append(chunk_job.index([job]))
                            
                            for count, chunk in enumerate(printing_chunks):
                                #Need print direction of chunk robot has just finished printing
                                print_direction_chunk[count] = print_direction[robot_schedules[moving_robots[count]][-2]]
                                initial_chunk_coordinates = chunk_positions[initial_chunks[chunk_job[robot_schedules[moving_robots[count]][-2]][0]]]
                                
                                if print_direction_chunk == 0:
                                    robot_starting_positions[count][0] = initial_chunk_coordinates[0]+1
                                    robot_starting_positions[count][1] = initial_chunk_coordinates[1]-1
                                elif print_direction_chunk == 1:
                                    robot_starting_positions[count][0] = initial_chunk_coordinates[0]+1
                                    robot_starting_positions[count][1] = initial_chunk_coordinates[1]+1
                                elif print_direction_chunk == 2:
                                    robot_starting_positions[count][0] = initial_chunk_coordinates[0]-1
                                    robot_starting_positions[count][1] = initial_chunk_coordinates[1]+1
                                else:
                                    robot_starting_positions[count][0] = initial_chunk_coordinates[0]-1
                                    robot_starting_positions[count][1] = initial_chunk_coordinates[1]-1
                                    
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
                            
                            visualizer.visualizer()
                            
                            #Now do all the stuff again
                            path_error = False
                            #rework the path to avoid diagonals and calculate path distance
                            (robot_path_lengths,robot_paths,robot_visualize_paths) = path_scrubber.scrub_paths(path)
                            
                            #visualize each step
                            # visualizer.visualizer()
                            
                            #calculate movement time for each robot
                            robot_move_time = ((robot_path_lengths)/(2))*tuning_variables.robot_speed + add_time
                            
                            # print("Robot Move Time " + str(robot_move_time))
                            
                            #add movement time to current action queue
                            for count, robot in enumerate(moving_robots):
                                time_current_action[robot] += (robot_move_time[count])
                            
                            # print("Time Current Action: " +str(time_current_action))
                            
                            #Find print time for all robots and add to time queue
                            printing_chunks_time = np.zeros(len(printing_chunks))
                            for i in range(0,len(printing_chunks)):
                                printing_chunks_time[i] = chunk_print_time[printing_chunks[i]]
                            
                            # print("Chunk Print Times "+ str(printing_chunks_time))    
                            
                            #add print time to current action queue
                            for count, robot in enumerate(moving_robots):
                                time_current_action[robot] += (printing_chunks_time[count])
                                
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
                            
                    except ValueError:
                        # print("Path Error")
                        path_error = True
                        chunk_print_states = [2]
                        elapsed_time = 1000
        
                
            else:
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
                    print("Debug Chunk Print States")
        else:
            print("All robots in use")
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
    
    


