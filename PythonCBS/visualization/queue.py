#This program is used to track the progress of the printing

"""
This program will track the steps of the printing process (printing and move
phase) and call the floor maker and cbs algorithm at each step of the process
"""
from typing import List, Tuple
import numpy as np

def queue(robot_starting_positions, chunk_info, floor_size):
    
    
    #Find all printable chunks
    printable_chunks = independent_chunks(chunk_info)
    
    #Find the robot position for all printable chunks
    printable_chunk_robot_positions = robot_chunk_positions(printable_chunks)
    
    #find robot goal positions
    #First decide which n chunks to print with n robots
    min_cost(robot_starting_positions, printable_chunk_robot_positions)
    
    robot_ending_positions = []
    total_print_time = 0
    return(robot_ending_positions, total_print_time)

def independent_chunks(chunk_info):
    #find which chunks have no dependencies
    for i  in range(0,len(chunk_info)):
        print(i)
    printable_chunks = []
    return(printable_chunks)

def robot_chunk_positions(printable_chunks):
    
    printable_chunk_robot_positions = []
    return(printable_chunk_robot_positions)
    

def min_cost(robot_positions, printable_chunk_robot_positions):
    # Finds the closest group of printable chunk positions given the robot positions
    sqdist = lambda x, y: (x[0]-y[0])**2 + (x[1]-y[1])**2
    cost_vec = []
    goal_positions = np.zeros(len(robot_positions))
    return(goal_positions)


#For testing of this method
robot_starting_positions = [[0,1],[1,0]]

#set dependencies, 4 jobs with 6 chunks each with 2 printable at a time
chunk_number = 24
chunk_print_time = np.round(np.random.rand(1,chunk_number)*10)
#all chunks on which a chunk is directly dependent
chunk_dependencies = [[],[],[],[],[],[],[],[],\
                      [0,1],[1],[2,3],[3],[4,5],[5],[6,7],[7],\
                      [8,9],[9],[10,11],[11],[12,13],[13],[14,15],[15]]
                      

chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]


#set job print direction 

job1_dir = 0
job2_dir = 0
job3_dir = 1
job4_dir = 3
job_directions = [job1_dir,job2_dir,job3_dir,job4_dir]
print_direction = np.zeros(chunk_number)
for i in range(0,chunk_number):
    print_direction[i] = job_directions[chunk_job[i][0]]
    

#chunk locations
chunk_positions = [[0,5],[1,5],[13,1],[14,1],[5,3],[5,2],[12,4],[12,5],\
                   [0,4],[1,4],[13,2],[14,2],[6,3],[6,2],[13,4],[13,5],\
                   [0,3],[1,3],[13,3],[14,3],[7,3],[7,2],[14,4],[14,5],]

chunk_info = [chunk_number,chunk_positions,chunk_print_time,chunk_dependencies,chunk_job,print_direction]

robot_starting_positions = [[0,0],[1,0],[2,0],[3,0]]

floor_size = [8,6]

(robot_ending_positions, total_print_time) =  queue(robot_starting_positions, chunk_info, floor_size)





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



