import numpy as np
import placement_visualizer
import scheduler

"""
This is the main method that does the placement optimization. This algorithm uses
a genetic algorithm to optimize placement position given a set of dependencies and
a floor size (in floor tiles)
THIS PLACEMENT METHOD IS ONLY APPLICABLE FOR RECTANGULAR JOBS THAT HAVE THE SAME NUMBER OF
CHUNKS IN EACH ROW AND COLUMN
"""

## inputs
#four equal jobs of 6 chunks
floor_size = [8,6]
chunk_dependencies = [[],[0],[],[2],[],[4],[],[6],\
                      [0],[1,8],[2],[3,10],[4],[5,12],[6],[7,14],\
                      [8],[9,16],[10],[11,18],[12],[13,20],[14],[15,22]]
chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]
chunk_print_time = [89., 82., 89., 84., 85., 80., 83., 87., 86., 81., 82., 87., 84., 64., 67., 78., \
        84., 94., 87., 49., 86., 89., 86., 83.]
    
#generate random placement
# def random_placement(floor_size, chunk_dependencies, chunk_job):
valid_positions = False
number_attempts = 0
while valid_positions == False:    
    valid_positions = True
    number_chunks = len(chunk_job)
    chunk_positions = [[]]*number_chunks
    restricted_positions = []
    number_jobs = np.max(chunk_job)+1
    number_chunks_in_job = np.zeros(number_jobs,int)
    for i in range(0,number_chunks):
        if chunk_job[i][0] == 0:
            number_chunks_in_job[0] += 1
        elif chunk_job[i][0] == 1:
            number_chunks_in_job[1] += 1
        elif chunk_job[i][0] == 2:
            number_chunks_in_job[2] += 1
        elif chunk_job[i][0] == 3:
            number_chunks_in_job[3] += 1
    
    #calculate floor size
    floor_x_max = floor_size[0]*2 - 1
    floor_y_max = floor_size[1]*2 -1
    
    #identify initial chunks
    initial_chunks = np.zeros(number_jobs,int) -1
    for job in range(0,number_jobs):
        chunk_counter = 0
        while initial_chunks[job] == -1:
            if chunk_dependencies[chunk_counter] == [] and chunk_job[chunk_counter][0] == job:
                initial_chunks[job] = chunk_counter
            chunk_counter += 1
    
    #identitfy all chunks in a job
    chunks_in_job = [[]]*number_jobs
    for chunk in range(0,number_chunks):
        chunks_in_job[chunk_job[chunk][0]] = chunks_in_job[chunk_job[chunk][0]] + [chunk]
    
    #generate random positions for initial chunks
    job_starting_posiitons = np.random.rand(number_jobs,2)
    job_starting_posiitons[:,0] *= floor_x_max
    job_starting_posiitons[:,1] *= floor_y_max
    job_starting_posiitons = (np.round(job_starting_posiitons)).astype(int)
    
    #place initial chunks for each job
    for job in range(0,number_jobs):
        chunk_positions[initial_chunks[job]] = [job_starting_posiitons[job][0], job_starting_posiitons[job][1]]
    
    
    #generate ramdom directions
    print_direction = (np.round(np.random.rand(1,number_jobs)*4)).astype(int)
    
    #place chunks for each job
    for job in range(0,number_jobs):
        #determine last independent chunk
        last_indep = -1
        chunk_counter = number_chunks_in_job[job] - 1
        while last_indep < 0:
            if chunk_dependencies[chunks_in_job[job][chunk_counter]] == []:
                last_indep = chunks_in_job[job][chunk_counter]
            chunk_counter -= 1
        
        #find row width
        row_width = 0
        if last_indep ==  initial_chunks[job]:
            #at most 2 wide
            if len(chunk_dependencies[chunks_in_job[job][3]]) > 1:
                row_width = 2
            else:
                row_width = 1
        else:
            if chunk_dependencies[chunks_in_job[job][(chunks_in_job[job].index(last_indep)+1)]] == [last_indep]:
                row_width = chunks_in_job[job].index(last_indep)+1 - chunks_in_job[job].index(initial_chunks[job]) +1
            else:
                row_width = chunks_in_job[job].index(last_indep) - chunks_in_job[job].index(initial_chunks[job]) +1
                
        #find column length
        column_length = int(len(chunks_in_job[job])/row_width)
        
        #set job direction as coordinates
        if print_direction[0][job] == 0:
            job_direction = [0,-1]
            trans_direction = [-1,0]
        elif print_direction[0][job] == 1:
            job_direction = [1,0]
            trans_direction = [0,-1]
        elif print_direction[0][job] == 2:
            job_direction = [0,1]
            trans_direction = [1,0]
        else:
            job_direction = [-1,0]
            trans_direction = [0,1]
        
        # print(chunk_positions[initial_chunks[job]])
        #place the other chunks
        chunk_counter = 0
        for row in range(0,column_length):
            for column in range(0,row_width):
                chunk_position = np.array(chunk_positions[initial_chunks[job]]) + np.array(trans_direction)*column + np.array(job_direction)*row
                # print(chunk_position)
                if chunk_counter != 0:
                    chunk_positions[chunks_in_job[job][chunk_counter]] = [chunk_position[0],chunk_position[1]]
                chunk_counter += 1
        
        #calculate restricted positions
        for row in range(0,column_length+1):
            for column in range(0,row_width):
                chunk_position = np.array(chunk_positions[initial_chunks[job]]) + np.array(trans_direction)*column + np.array(job_direction)*row
                restricted_positions.append([chunk_position[0],chunk_position[1]])
                
                #check for validity
                if chunk_position[0] < 0 or chunk_position[0] > floor_x_max or chunk_position[1] < 0 or chunk_position[1] > floor_y_max: 
                    valid_positions = False
                
    #check to make sure there is no overlap including with positions needed to print each job
    coordinates_set = np.unique(np.array(restricted_positions), axis = 0)
    
    if len(coordinates_set) == len(restricted_positions) and valid_positions == True:
        valid_positions = True
    else:
        valid_positions = False
        
    # print(valid_positions)
    number_attempts += 1
                
placement_visualizer.placement_vis(floor_size, chunk_positions, chunk_job)
print(number_attempts)

# return(chunk_positions)




#view placement

#genetic algorithm


#call scheduler to get results