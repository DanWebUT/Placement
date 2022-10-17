import numpy as np
import placement_visualizer
import scheduler
import copy


"""
This is the main method that does the placement optimization. This algorithm uses
a genetic algorithm to optimize placement position given a set of dependencies and
a floor size (in floor tiles)
THIS PLACEMENT METHOD IS ONLY APPLICABLE FOR RECTANGULAR JOBS THAT HAVE THE SAME NUMBER OF
CHUNKS IN EACH ROW AND COLUMN
"""
## inputs
#four equal jobs of 6 chunks

chunk_dependencies = [[],[0],[],[2],[],[4],[],[6],\
                      [0],[1,8],[2],[3,10],[4],[5,12],[6],[7,14],\
                      [8],[9,16],[10],[11,18],[12],[13,20],[14],[15,22]]
chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]
chunk_print_time = [89., 82., 89., 84., 85., 80., 83., 87., 86., 81., 82., 87., 84., 64., 67., 78., \
        84., 94., 87., 49., 86., 89., 86., 83.]
robot_starting_positions = [[0,0],[1,0],[2,0],[3,0]]
floor_size = [8,6]

def find_initial_chunks(number_jobs, chunk_dependencies, chunk_job):
    #identify initial chunks
    initial_chunks = np.zeros(number_jobs,int) -1
    for job in range(0,number_jobs):
        chunk_counter = 0
        while initial_chunks[job] == -1:
            if chunk_dependencies[chunk_counter] == [] and chunk_job[chunk_counter][0] == job:
                initial_chunks[job] = chunk_counter
            chunk_counter += 1
    return(initial_chunks)    
    
def num_chunks_in_job(number_jobs,chunk_job):
    number_chunks = len(chunk_job)    
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
    return(number_chunks_in_job)

def identify_chunks_in_job(number_jobs, number_chunks, chunk_job):
    #identitfy all chunks in a job
    chunks_in_job = [[]]*number_jobs
    for chunk in range(0,number_chunks):
        chunks_in_job[chunk_job[chunk][0]] = chunks_in_job[chunk_job[chunk][0]] + [chunk]
    return(chunks_in_job)

#generate random placement
def random_placement(floor_size, chunk_dependencies, chunk_job):
    number_jobs = np.max(chunk_job)+1
    
    #calculate floor size
    floor_x_max = floor_size[0]*2 - 1
    floor_y_max = floor_size[1]*2 -1
    
    #generate random positions for initial chunks
    job_starting_posiitons = np.random.rand(number_jobs,2)
    job_starting_posiitons[:,0] *= floor_x_max
    job_starting_posiitons[:,1] *= floor_y_max
    job_starting_posiitons = (np.round(job_starting_posiitons)).astype(int)
    
    #generate ramdom directions
    print_direction = (np.round(np.random.rand(1,number_jobs)*3)).astype(int)

    return(job_starting_posiitons, print_direction)
  
def place_chunks(job_starting_posiitons, print_direction, chunk_job, chunk_dependencies, floor_size, robot_starting_positions):
    number_chunks = len(chunk_job)
    number_jobs = np.max(chunk_job)+1
    chunk_positions = [[]]*number_chunks
    initial_chunks = find_initial_chunks(number_jobs, chunk_dependencies, chunk_job)
    
    number_chunks_in_job = num_chunks_in_job(number_jobs,chunk_job)
    chunks_in_job = identify_chunks_in_job(number_jobs, number_chunks, chunk_job)
    
    restricted_positions = []
    valid_positions = True
    
    #calculate floor size
    floor_x_max = floor_size[0]*2 - 1
    floor_y_max = floor_size[1]*2 -1
    
    #place initial chunks for each job
    for job in range(0,number_jobs):
        chunk_positions[initial_chunks[job]] = [job_starting_posiitons[job][0], job_starting_posiitons[job][1]]
    
    
    
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
            for column in range(0,row_width+1):
                chunk_position = np.array(chunk_positions[initial_chunks[job]]) + np.array(trans_direction)*column + np.array(job_direction)*row
                restricted_positions.append([chunk_position[0],chunk_position[1]])
                
                #check for validity
                if chunk_position[0] < 0 or chunk_position[0] > floor_x_max or chunk_position[1] < 0 or chunk_position[1] > floor_y_max: 
                    valid_positions = False
    
    #restrict from robot starting positions
    for robot in range(0, len(robot_starting_positions)):
        restricted_positions.append([robot_starting_positions[robot][0],robot_starting_positions[robot][1]])
    
    #check to make sure there is no overlap including with positions needed to print each job
    coordinates_set = np.unique(np.array(restricted_positions), axis = 0)
    
    if len(coordinates_set) == len(restricted_positions) and valid_positions == True:
        valid_positions = True
    else:
        valid_positions = False
        
    # print(valid_positions
                
    
    
    return(chunk_positions, valid_positions)

def parent_probabilities(num_selection, print_time_pop):
    #Adapted from https://pub.towardsai.net/genetic-algorithm-ga-introduction-with-example-code-e59f9bc58eaf
    #set probabilites of each parent being selected
    parent_cost_list = print_time_pop[:num_selection]
    avg_cost = np.mean(parent_cost_list)
    beta = 1
    parent_cost_list = np.array(parent_cost_list)/avg_cost
    probabilities = np.exp(-beta*np.array(parent_cost_list))
    return(probabilities)
    
def select_parent(probabilities):
    #select parent and return index based on probabilities
    c = np.cumsum(probabilities)
    r = sum(probabilities) * np.random.rand()
    ind = np.argwhere(r <= c)
    return ind[0][0]

def list_genes(individual):
    gene = []
    num_jobs = len(individual[0])
    for job in range(0, num_jobs):
        for coordinate in range(0,2):
            gene.append(individual[0][job][coordinate])
        gene.append(individual[1][job])
        
    return(gene)

def crossover(p1_genes, p2_genes):
    gene_length = len(p1_genes)
    crossover_location = int(np.floor(np.random.rand()*gene_length))
    part1 = p1_genes[:crossover_location]
    part2 = p2_genes[crossover_location:]
    
    child = part1 + part2
    return(child)

def gene_to_tuple(child):
    num_jobs = int(len(child)/3)
    job_starting_posiitons = np.zeros((num_jobs,2))
    job_direction = [0]*num_jobs
    for job in range(0,num_jobs):
        job_starting_posiitons[job][0] = child[job*3]
        job_starting_posiitons[job][1] = child[job*3+1]
        job_direction[job] = child[job*3+2]
    return(job_starting_posiitons, job_direction)
    
if __name__ == '__main__':
    #GA parameters
    num_pop = 10
    percent_crossover = .3
    percent_mutation = .2
    num_iterations = 10
    percent_selection = .4
    
    
    # #find valid configurations
    # print_time_pop = [[]]*num_pop
    # population_tuple = []
    
    # for individual in range(0,10):
    #     chunk_dep_iteration = copy.deepcopy(chunk_dependencies)
    #     valid_positions = False
    #     number_attempts = 0
    #     while valid_positions == False:    
    #         (job_starting_posiitons, job_directions) = random_placement(floor_size, chunk_dep_iteration, chunk_job)
    #         (chunk_positions, valid_positions) = place_chunks(job_starting_posiitons, job_directions, chunk_job, chunk_dep_iteration, floor_size, robot_starting_positions)
    #         number_attempts += 1
    #         job_directions = job_directions[0]
    #         if valid_positions == True:
    #             print_direction = scheduler.chunk_print_direction(job_directions, chunk_job)     
    #             (total_print_time, path_error) = scheduler.schedule(robot_starting_positions, floor_size, chunk_dep_iteration, chunk_job, chunk_print_time, chunk_positions, print_direction)
    #             if path_error == True:
    #                 valid_positions = False
        
    #     #evaluate results
    #     print("Total Print Time: " + str(total_print_time))
            
    #     print_time_pop[individual] = total_print_time  
    #     iteration_tuple = (job_starting_posiitons, job_directions, total_print_time)
    #     population_tuple.append(iteration_tuple)
        
    # sorted(population_tuple, key = lambda individual: individual[2])
    
    print_time_pop = [534.3500000000001,
     534.5500000000001,
     534.75,
     534.75,
     534.1500000000001,
     534.5500000000001,
     534.1500000000001,
     534.5500000000001,
     534.5500000000001,
     534.3500000000001]
        
    sorted_population_tuple = [(np.array([[0, 2],
             [4, 2],
             [8, 4],
             [5, 3]]),
      np.array([2, 1, 1, 2]),
      533.5500000000001),
     (np.array([[ 4,  9],
             [ 6,  5],
             [ 1,  3],
             [10,  2]]),
      np.array([3, 0, 2, 1]),
      533.75),
     (np.array([[13,  3],
             [ 5,  7],
             [ 4, 10],
             [ 3,  4]]),
      np.array([0, 1, 0, 0]),
      533.95),
     (np.array([[14,  3],
             [ 4,  4],
             [ 4,  5],
             [10,  2]]),
      np.array([3, 1, 3, 1]),
      534.1500000000001),
     (np.array([[ 9,  5],
             [11,  2],
             [ 6,  2],
             [ 1,  9]]),
      np.array([2, 1, 1, 1]),
      534.1500000000001),
     (np.array([[ 2,  2],
             [13,  2],
             [ 4,  9],
             [ 9,  4]]),
      np.array([2, 2, 1, 1]),
      534.1500000000001),
     (np.array([[9, 6],
             [6, 2],
             [2, 8],
             [4, 7]]),
      np.array([2, 1, 0, 2]),
      534.1500000000001),
     (np.array([[12,  3],
             [ 2, 10],
             [ 0,  4],
             [ 9,  9]]),
      np.array([1, 0, 1, 0]),
      534.5500000000001),
     (np.array([[ 4,  8],
             [ 3,  7],
             [10,  6],
             [11,  1]]),
      np.array([2, 3, 2, 2]),
      534.5500000000001),
     (np.array([[ 3, 10],
             [ 6,  6],
             [10,  8],
             [ 9,  5]]),
      np.array([0, 0, 1, 1]),
      534.75)]
    
    print_time_pop.sort()
    
    #genetic algorithm
    #find best results for selection and carry those over to new population
    num_selection = int(np.round(num_pop*percent_selection))
    new_popilation = sorted_population_tuple[:num_selection]
    
    
    #create remaining from these
    num_new_individuals =  num_pop-num_selection
    probabilites = parent_probabilities(num_selection, print_time_pop)
        
    for individual in range(0,1):
        valid_positions = False
        while valid_positions == False:            
            p1_index = select_parent(probabilites)
            p2_index = select_parent(probabilites)
            while p2_index == p1_index:
                p2_index = select_parent(probabilites)
            
            p1_genes = list_genes(sorted_population_tuple[p1_index])
            p2_genes = list_genes(sorted_population_tuple[p2_index])
        
            #crossover
            child = crossover(p1_genes, p2_genes)
            
            #check validity 
            chunk_dep_iteration = copy.deepcopy(chunk_dependencies)
            (child_job_starting_positions, child_job_directions) = gene_to_tuple(child)
            (chunk_positions, valid_positions) = place_chunks(child_job_starting_positions, [child_job_directions], chunk_job, chunk_dep_iteration, floor_size, robot_starting_positions)
            if valid_positions == True:
                print_direction = scheduler.chunk_print_direction(child_job_directions, chunk_job)     
                (total_print_time, path_error) = scheduler.schedule(robot_starting_positions, floor_size, chunk_dep_iteration, chunk_job, chunk_print_time, chunk_positions, print_direction)
                if path_error == True:
                    valid_positions = False
        
        print("Parents: " + str(p1_index) + " and " + str(p2_index))
        print(total_print_time)
        #add to new population once validity is confirmed
        iteration_tuple = (child_job_starting_positions, child_job_directions, total_print_time)
        new_popilation.append(iteration_tuple)
        print_time_pop[num_selection+individual] = total_print_time
            
    print(print_time_pop)
    
    
#view placement    
# print("Number of Attempts: " + str(number_attempts))
# placement_visualizer.placement_vis(floor_size, chunk_positions, chunk_job)
    




#genetic algorithm


#call scheduler to get results