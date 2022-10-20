import numpy as np
import placement_visualizer
import scheduler
import copy
import os
import time


"""
This is the main method that does the placement optimization. This algorithm uses
a genetic algorithm to optimize placement position given a set of dependencies and
a floor size (in floor tiles)
THIS PLACEMENT METHOD IS ONLY APPLICABLE FOR RECTANGULAR JOBS THAT HAVE THE SAME NUMBER OF
CHUNKS IN EACH ROW AND COLUMN
"""
## inputs
#four equal jobs of 6 chunks

# chunk_dependencies = [[],[0],[],[2],[],[4],[],[6],\
#                       [0],[1,8],[2],[3,10],[4],[5,12],[6],[7,14],\
#                       [8],[9,16],[10],[11,18],[12],[13,20],[14],[15,22]]
# chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]
# chunk_print_time = [89., 82., 89., 84., 85., 80., 83., 87., 86., 81., 82., 87., 84., 64., 67., 78., \
#         84., 94., 87., 49., 86., 89., 86., 83.]
# robot_starting_positions = [[0,0],[1,0],[2,0],[3,0]]
# floor_size = [8,6]

#four equal jobs of 6 chunks rotated 90 degrees
chunk_dependencies = [[],[0, 2],[],[],[3, 5],[],[],[6, 8], [],[],[9, 11],[],\
                      [0],[1,12,14],[2],[3],[4,15,17],[5],[6],[7,18,20],[8],[9],[10,21,23],[11]]
    
chunk_job = [[0],[0],[0],[1],[1],[1],[2],[2],[2],[3],[3],[3],[0],[0],[0],[1],[1],[1],[2],[2],[2],[3],[3],[3]]

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
            try:
                if chunk_dependencies[chunks_in_job[job][(chunks_in_job[job].index(last_indep)+1)]] == [last_indep]:
                    row_width = chunks_in_job[job].index(last_indep)+1 - chunks_in_job[job].index(initial_chunks[job]) +1
                else:
                    row_width = chunks_in_job[job].index(last_indep) - chunks_in_job[job].index(initial_chunks[job]) +1
            except IndexError:
                print("What went wrong")
                
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

def parent_probabilities(num_parents, print_time_pop):
    #Adapted from https://pub.towardsai.net/genetic-algorithm-ga-introduction-with-example-code-e59f9bc58eaf
    #set probabilites of each parent being selected
    parent_cost_list = print_time_pop[:num_parents]
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

def create_random_configuration(floor_size, chunk_dependencies, chunk_job, robot_starting_positions, chunk_print_time): 
    valid_positions = False
    number_attempts = 0
    while valid_positions == False:
        chunk_dep_iteration = copy.deepcopy(chunk_dependencies)
        print(number_attempts)
        (job_starting_posiitons, job_directions) = random_placement(floor_size, chunk_dep_iteration, chunk_job)
        (chunk_positions, valid_positions) = place_chunks(job_starting_posiitons, job_directions, chunk_job, chunk_dep_iteration, floor_size, robot_starting_positions)
        number_attempts += 1
        job_directions = job_directions[0]
        if valid_positions == True:
            print_direction = scheduler.chunk_print_direction(job_directions, chunk_job)     
            (total_print_time, path_error) = scheduler.schedule(robot_starting_positions, floor_size, chunk_dep_iteration, chunk_job, chunk_print_time, chunk_positions, print_direction)
            if path_error == True:
                valid_positions = False
    
    #For visualization
    placement_visualizer.placement_vis(floor_size, chunk_positions, chunk_job)
    
    return(total_print_time, job_starting_posiitons, job_directions)
                    
if __name__ == '__main__':
    #GA parameters
    num_pop = 10
    percent_mutation = .5
    num_generations = 100
    percent_carryover = .2
    percent_crossover = .6
    
    """
    The GA is set up in a way where the best percent carryover of the population is carried
    to the next generation. Then the next percent crossover of the new population is created 
    by crossover. These also have a chance of mutation and can be made up of non-unique parents
    Finally, the remaining members are randomly generated
    """
    #Write to a folder
    time_setting = int(np.round((time.time())/1000))
    folder = "GA_Results/GA" + str(time_setting)

    current_path = os.getcwd()
    filepath = os.path.join(current_path,folder)
    os.makedirs(filepath, exist_ok=True)

    filename = "Print_Times"
    config_filename = "Configurations"
    complete_filename = os.path.join(filepath, filename +".txt")
    config_complete_filename = os.path.join(filepath, config_filename +".txt")
    
    #find valid configurations
    print_time_pop = [[]]*num_pop
    population_tuple = []
    
    for individual in range(0,num_pop):
        chunk_dep_iteration = copy.deepcopy(chunk_dependencies)
        print(chunk_dependencies)
        print(chunk_dep_iteration)
        (total_print_time, job_starting_posiitons, job_directions) = create_random_configuration(floor_size, chunk_dep_iteration, chunk_job, robot_starting_positions, chunk_print_time)
        
        print(total_print_time)
        
        #evaluate results    
        print_time_pop[individual] = total_print_time  
        iteration_tuple = (job_starting_posiitons, job_directions, total_print_time)
        population_tuple.append(iteration_tuple)
        
    sorted_population_tuple = sorted(population_tuple, key = lambda individual: individual[2])
    
    print_time_pop.sort() 

    print("Original population print time: \n" + str(print_time_pop))
    #create empty file
    with open(complete_filename, "w") as file:
        file.write("Original population print time: \n" + str(print_time_pop) + "\n")
    
    with open(config_complete_filename, "w") as file:
        file.write("Original population configuration: \n" + str(sorted_population_tuple) + "\n")
    
    #genetic algorithm
    #find best results for selection and carry those over to new population
    num_carryover = int(np.round(num_pop*percent_carryover))
    num_crossover = int(np.round(num_pop*percent_crossover))
    num_parents = num_carryover+num_crossover
    num_new_random = num_pop-num_carryover-num_crossover
    
    for generation in range(1,num_generations+1):
        new_population = sorted_population_tuple[:num_carryover]
    
        #create remaining from these
        probabilites = parent_probabilities(num_parents, print_time_pop)
        for individual in range(0,int(num_crossover)):
            valid_positions = False
            
            #make sure child is valid configuration and that it forms in less than 100 iterations
            iteration_count = 0
            while valid_positions == False and iteration_count < 100:            
                p1_index = select_parent(probabilites)
                p2_index = select_parent(probabilites)
                #index can be the same but this is intentional to allow for mutation of
                #the original parents without crossover in the new population
                
                p1_genes = list_genes(sorted_population_tuple[p1_index])
                p2_genes = list_genes(sorted_population_tuple[p2_index])
            
                #crossover
                child = crossover(p1_genes, p2_genes)
                
                #mutation
                mutation = False
                if np.random.rand() < percent_mutation:
                    mutation = True
                    #generate random location
                    mutation_location = int(np.floor(np.random.rand()*len(child)))
                    
                    #generate +1 or -1 change at location
                    if np.random.rand() < .5:
                        change = -1
                    else:
                        change = +1
                    
                    child[mutation_location] = child[mutation_location] + change
                    
                    #make sure direction is in range 0 to 3
                    if (mutation_location+1)%3 == 0 and (child[mutation_location] < 0 or child[mutation_location] > 3):
                        child[mutation_location] = child[mutation_location]%4
                    
                #check validity 
                chunk_dep_iteration = copy.deepcopy(chunk_dependencies)
                (child_job_starting_positions, child_job_directions) = gene_to_tuple(child)
                (chunk_positions, valid_positions) = place_chunks(child_job_starting_positions, [child_job_directions], chunk_job, chunk_dep_iteration, floor_size, robot_starting_positions)
                if valid_positions == True:
                    print_direction = scheduler.chunk_print_direction(child_job_directions, chunk_job)     
                    (total_print_time, path_error) = scheduler.schedule(robot_starting_positions, floor_size, chunk_dep_iteration, chunk_job, chunk_print_time, chunk_positions, print_direction)
                    if path_error == True:
                        valid_positions = False
                
                iteration_count += 1
            
            #if child does not form in 100 iterations, pick parent one to move to next generation
            if iteration_count >= 100:
                iteration_tuple = sorted_population_tuple[p1_index]
                
                #FOR TESTING
                # print("Iteration count exceeded, carry " + str(p1_index) + " to next generation")
                # print("Mutation is " + str(mutation))
                # print(total_print_time)
            else:
                iteration_tuple = (child_job_starting_positions, np.array(child_job_directions), total_print_time)
                
                #FOR TESTING
                # print("Parents: " + str(p1_index) + " and " + str(p2_index))
                # print("Mutation is " + str(mutation))
                # print(total_print_time)
    
            #add to new population once validity is confirmed
            new_population.append(iteration_tuple)
            print_time_pop[num_carryover+individual] = total_print_time
        
        for individual in range(0,int(num_new_random)):
            (total_print_time, job_starting_posiitons, job_directions) = create_random_configuration(floor_size, chunk_dependencies, chunk_job, robot_starting_positions, chunk_print_time)
            
            # print(total_print_time)
            
            #evaluate results    
            iteration_tuple = (job_starting_posiitons, job_directions, total_print_time)
            new_population.append(iteration_tuple)
            print_time_pop[num_carryover+num_crossover+individual] = total_print_time
        
        print_time_pop.sort()
        new_population = sorted(new_population, key = lambda individual: individual[2])
        
        population_tuple = copy.deepcopy(new_population)
    
        #FOR TESTING
        print("Generation " +str(generation) +" population print time: \n" + str(print_time_pop))
        
        #Write print and configuration results
        with open(complete_filename, "a+") as file:
           file.write("Generation " +str(generation) +" population print time: \n" + str(print_time_pop) + "\n")
           
        with open(config_complete_filename, "a+") as file:
            file.write("Generation " +str(generation) +" population configuration: \n" + str(new_population) + "\n")
    
#view placement    
# print("Number of Attempts: " + str(number_attempts))
# placement_visualizer.placement_vis(floor_size, chunk_positions, chunk_job)
    




#genetic algorithm


#call scheduler to get results