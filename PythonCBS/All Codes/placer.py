from numpy import round
from numpy import mean
from numpy import array
from numpy import cumsum
from numpy import argwhere
from numpy import exp
from numpy import floor
from numpy import zeros
from numpy import max
from numpy import unique
from numpy.random import rand
from numpy.linalg import norm

from copy import deepcopy
from time import time

from scheduler import chunk_print_direction
from scheduler import schedule

import placement_visualizer
import os



"""
This is the main method that does the placement optimization. This algorithm uses
a genetic algorithm to optimize placement position given a set of dependencies and
a floor size (in floor tiles)
THIS PLACEMENT METHOD IS ONLY APPLICABLE FOR RECTANGULAR JOBS THAT HAVE THE SAME NUMBER OF
CHUNKS IN EACH ROW AND COLUMN
"""
# # inputs
# four equal jobs of 6 chunks
# chunk_dependencies = [[],[0],[],[2],[],[4],[],[6],\
#                       [0],[1,8],[2],[3,10],[4],[5,12],[6],[7,14],\
#                       [8],[9,16],[10],[11,18],[12],[13,20],[14],[15,22]]
# chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]
# chunk_print_time = [2253., 1899., 2253., 1899., 2253., 1899., 2253., 1899.,\
#                     2929., 2490., 2929., 2490., 2929., 2490., 2929., 2490.,  \
#                     1429., 1236., 1429., 1236., 1429., 1236., 1429., 1236.]
# robot_starting_positions = [[0,0],[1,0],[2,0],[3,0]]
# floor_size = [8,6]

# # #four equal jobs of 6 chunks rotated 90 degrees
# chunk_dependencies = [[],[0, 2],[],[],[3, 5],[],[],[6, 8], [],[],[9, 11],[],\
#                       [0],[1,12,14],[2],[3],[4,15,17],[5],[6],[7,18,20],[8],[9],[10,21,23],[11]]
    
# chunk_job = [[0],[0],[0],[1],[1],[1],[2],[2],[2],[3],[3],[3],[0],[0],[0],[1],[1],[1],[2],[2],[2],[3],[3],[3]]

# chunk_print_time = [2253., 2859., 1552., 2253., 2859., 1552., 2253., 2859., 1552., 2253., 2859., 1552., \
#         1894, 2598, 1194, 1894, 2598, 1194, 1894, 2598, 1194, 1894, 2598, 1194]
# robot_starting_positions = [[0,0],[1,0],[2,0],[3,0]]
# floor_size = [8,6]

# ##Pyramid
# chunk_dependencies = [[],[0,2],[],[],[3,5],[],[],[6],[], \
#                       [0],[1,7,11],[2],[3],[4,12,14],[5],[6],[7,15],[8], \
#                       [9],[10,18,20],[11],[12],[13,21,23],[14],[15],[16,24], \
#                       [18],[19,26,28],[20],[21],[22,29,31],[23], \
#                       [26],[27,32,34],[28]]
# chunk_job = [[0],[0],[0],[1],[1],[1],[2],[2],[3],\
#              [0],[0],[0],[1],[1],[1],[2],[2],[3],\
#              [0],[0],[0],[1],[1],[1],[2],[2], \
#              [0],[0],[0],[1],[1],[1],\
#              [0],[0],[0]]

# chunk_print_time = [1573.,2209.,1220.,1579.,1937.,158.,1573.,776.,977.,\
#               2519.,3847.,2021.,2528.,3306.,266.,2450.,1251.,141.,\
#               2070.,3163.,1662.,2079.,2763.,174.,166.,73., \
#               2068.,3163.,1662.,528.,649.,29.,\
#               929.,1274.,777.]
# floor_size = [8,6]
# robot_starting_positions = [[0,0],[1,0],[2,0],[3,0]]

#Resized Benchy
chunk_dependencies = [[],[0],[],[2],[],[4],[],[6],\
                      [0],[1,8],[2],[3,10],[4],[5,12],\
                      [8],[9,14],[10],[11,16]]
chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[0],[0],[1],[1]]
chunk_print_time = [1152., 655., 1075., 722., 279., 86., 1139., 81.,\
                    2039., 1496., 716., 564., 561., 146.,  \
                    561., 217., 8., 8.]
robot_starting_positions = [[0,0],[1,0],[2,0],[3,0]]
floor_size = [8,6]

# job_directions = [2,2,2,2]
# chunk_positions = [[1,1],[2,1],[3,1],[6,1],[7,1],[8,1],[11,1],[12,1],[12,8],\
#              [1,2],[2,2],[3,2],[6,2],[7,2],[8,2],[11,2],[12,2],[12,9],\
#              [1,3],[2,3],[3,3],[6,3],[7,3],[8,3],[11,3],[12,3], \
#              [1,4],[2,4],[3,4],[6,4],[7,4],[8,4],\
#              [1,5],[2,5],[3,5],]
    
def find_initial_chunks(number_jobs, chunk_dependencies, chunk_job):
    #identify initial chunks
    initial_chunks = zeros(number_jobs,int) -1
    for job in range(0,number_jobs):
        chunk_counter = 0
        while initial_chunks[job] == -1:
            if chunk_dependencies[chunk_counter] == [] and chunk_job[chunk_counter][0] == job:
                initial_chunks[job] = chunk_counter
            chunk_counter += 1
    return(initial_chunks)    
    
def num_chunks_in_job(number_jobs,chunk_job):
    number_chunks = len(chunk_job)    
    number_chunks_in_job = zeros(number_jobs,int)
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
    number_jobs = max(chunk_job)+1
    
    #calculate floor size
    floor_x_max = floor_size[0]*2 - 1
    floor_y_max = floor_size[1]*2 -1
    
    #generate random positions for initial chunks
    job_starting_posiitons = rand(number_jobs,2)
    job_starting_posiitons[:,0] *= floor_x_max
    job_starting_posiitons[:,1] *= floor_y_max
    job_starting_posiitons = (round(job_starting_posiitons)).astype(int)
    
    #generate ramdom directions
    print_direction = (round(rand(1,number_jobs)*3)).astype(int)

    return(job_starting_posiitons, print_direction)
  
def place_chunks(job_starting_posiitons, print_direction, chunk_job, chunk_dependencies, floor_size, robot_starting_positions):
    number_chunks = len(chunk_job)
    number_jobs = max(chunk_job)+1
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
            if len(chunks_in_job[job]) > 2:
                #at most 2 wide
                if len(chunk_dependencies[chunks_in_job[job][3]]) > 1:
                    row_width = 2
                else:
                    row_width = 1
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
                chunk_position = array(chunk_positions[initial_chunks[job]]) + array(trans_direction)*column + array(job_direction)*row
                # print(chunk_position)
                if chunk_counter != 0:
                    chunk_positions[chunks_in_job[job][chunk_counter]] = [chunk_position[0],chunk_position[1]]
                chunk_counter += 1
        
        #calculate restricted positions
        #there is a one wide gap on the sides
        for row in range(0,column_length+1):
            for column in range(-1,row_width+1):
                chunk_position = array(chunk_positions[initial_chunks[job]]) + array(trans_direction)*column + array(job_direction)*row
                restricted_positions.append([chunk_position[0],chunk_position[1]])
                
                #check for validity
                if chunk_position[0] < 0 or chunk_position[0] > floor_x_max or chunk_position[1] < 0 or chunk_position[1] > floor_y_max: 
                    valid_positions = False
    
    
    #make sure the jobs satisfy assembly order
    distance_matrix = norm(job_starting_posiitons - job_starting_posiitons[:,None],axis =-1)
    for job in range(1,len(job_starting_posiitons)):
        if distance_matrix[job][0] < distance_matrix[job-1][0]:
            valid_positions = False
            
        #FOR TESTING
        # print(valid_positions)
    
    # #restrict from robot starting positions
    # for robot in range(0, len(robot_starting_positions)):
    #     restricted_positions.append([robot_starting_positions[robot][0],robot_starting_positions[robot][1]])
    
    #check to make sure there is no overlap including with positions needed to print each job
    coordinates_set = unique(array(restricted_positions), axis = 0)
    
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
    avg_cost = mean(parent_cost_list)
    beta = 10
    parent_cost_list = array(parent_cost_list)/avg_cost
    probabilities = exp(-beta*array(parent_cost_list))
    return(probabilities)
    
def select_parent(probabilities):
    #select parent and return index based on probabilities
    c = cumsum(probabilities)
    r = sum(probabilities) * rand()
    ind = argwhere(r <= c)
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
    crossover_location = int(floor(rand()*gene_length))
    part1 = p1_genes[:crossover_location]
    part2 = p2_genes[crossover_location:]
    
    child = part1 + part2
    return(child)

def gene_to_tuple(child):
    num_jobs = int(len(child)/3)
    job_starting_posiitons = zeros((num_jobs,2))
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
        chunk_dep_iteration = deepcopy(chunk_dependencies)
        
        #FOR TESTING
        # print(number_attempts)
        
        (job_starting_posiitons, job_directions) = random_placement(floor_size, chunk_dep_iteration, chunk_job)
        (chunk_positions, valid_positions) = place_chunks(job_starting_posiitons, job_directions, chunk_job, chunk_dep_iteration, floor_size, robot_starting_positions)
        number_attempts += 1
        job_directions = job_directions[0]
        if valid_positions == True:
            try:
                print_direction = chunk_print_direction(job_directions, chunk_job)     
                (total_print_time, path_error) = schedule(robot_starting_positions, floor_size, chunk_dep_iteration, chunk_job, chunk_print_time, chunk_positions, print_direction)
                if path_error == True:
                    valid_positions = False
            except IndexError:
                valid_positions = False
    
    #For visualization
    # placement_visualizer.placement_vis(floor_size, chunk_positions, chunk_job)
    
    return(total_print_time, job_starting_posiitons, job_directions)

def convergence_test(print_time_pop, num_pop, percent_random):
    convergence_criteria = .00001 #percentage for convergence
    # convergence_criteria = 5 #range for convergence in values
    
    num_pop_convergence = int(num_pop-num_pop*(percent_random))
    
    convergence_pop = print_time_pop[:num_pop_convergence]
    average = mean(convergence_pop)
    
    for individual in convergence_pop:
        if individual >= average*(1-convergence_criteria/2) and individual <= average*(1+convergence_criteria/2):
            convergence = True
        else:
            convergence = False
            return(convergence)
        
    # for individual in convergence_pop:
    #     if individual >= average-convergence_criteria/2 and individual <= average+convergence_criteria/2:
    #         convergence = True
    #     else:
    #         convergence = False
    #         return(convergence)

    return(convergence)    

if __name__ == '__main__':
    #GA parameters
    num_pop = 20
    chance_mutation = .1
    chance_crossover = .4
    # num_generations = 100
    percent_elite = .3
    percent_random = .3 #percent of new randomly generated populateion
    
    """
    The GA is set up in a way where the best percent elite of the population is carried
    to the next generation. Then the next percent crossover of the new population is created 
    by crossover. These also have a chance of mutation and can be made up of non-unique parents
    Finally, the remaining members are randomly generated
    """
    #Write to a folder
    time_setting = int(round((time())/1000))
    folder = "GA_Results/Tall_box_Final" + str(time_setting)

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
        chunk_dep_iteration = deepcopy(chunk_dependencies)
        
        #FOR TESTING
        # print(chunk_dependencies)
        # print(chunk_dep_iteration)
        
        (total_print_time, job_starting_posiitons, job_directions) = create_random_configuration(floor_size, chunk_dep_iteration, chunk_job, robot_starting_positions, chunk_print_time)
        
        #FOR TESTING
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
    num_elite = int(round(num_pop*percent_elite))
    num_crossover = int(round(num_pop-num_pop*(percent_elite+percent_random)))
    # num_parents = num_elite
    num_new_random = int(num_pop*percent_random)
    
    
    convergence = convergence_test(print_time_pop, num_pop, 0)
    
    
    generation = 1
    #setup convergence
    while convergence == False and generation <=100:
        new_population = sorted_population_tuple[:num_elite]
    
        #create remaining from these
        probabilites = parent_probabilities(num_pop, print_time_pop)
        for individual in range(0,int(num_crossover)):
            valid_positions = False
            
            #make sure child is valid configuration and that it forms in less than 100 iterations
            iteration_count = 0
            while valid_positions == False and iteration_count < 50:            
                #crossover
                if rand() < chance_crossover:
                    p1_index = select_parent(probabilites)
                    p2_index = select_parent(probabilites)
                    #make sure parents are unique
                    while p1_index == p2_index:
                        p2_index = select_parent(probabilites)
                    
                    p1_genes = list_genes(sorted_population_tuple[p1_index])
                    p2_genes = list_genes(sorted_population_tuple[p2_index])
                    
                    child = crossover(p1_genes, p2_genes)
                
                else:
                    child_index = select_parent(probabilites)
                    child = list_genes(sorted_population_tuple[child_index])
                
                #mutation
                mutation = False
                if rand() < chance_mutation:
                    #choose either single point mutation, or all of one type of gener
                    if rand() < .7:
                        #single point mutation
                        mutation = True
                        #generate random location
                        mutation_location = int(floor(rand()*len(child)))
                        
                        if rand() < .5:
                            new_val = child[mutation_location] + 1
                        else:
                            new_val = child[mutation_location] - 1
                        #generate change within bounds of that number
                        if (mutation_location)%3 == 0 and (new_val >= (floor_size[0]*2) or new_val < 0):
                            new_val = int(new_val%(floor_size[0]*2))
                        elif (mutation_location+2)%3 == 0 and (new_val >= (floor_size[1]*2) or new_val < 0):
                            new_val = int(new_val%(floor_size[1]*2))
                        elif (mutation_location+1)%3 == 0 and (new_val >= 4 or new_val < 0):
                            new_val = int(new_val%4)
                        
                        
                        # print("Mutation at " + str(mutation_location) + " from " +str(child[mutation_location])+" to " +str(new_val))
                        child[mutation_location] = new_val
                        
                    else:
                        mutation = True
                        #figure out which of 3 gene types to vary
                        mutation_gene = int(floor(rand()*3))
                        #set change up or down
                        if rand() < .5: 
                            change = 1
                        else:
                            change = -1
                        
                        #set each gene in new robot
                        for gene in range(0,len(robot_starting_positions)):
                            mutation_location = mutation_gene + gene*3
                            child[mutation_location] = child[mutation_location] + change
                    
                #check validity 
                chunk_dep_iteration = deepcopy(chunk_dependencies)
                (child_job_starting_positions, child_job_directions) = gene_to_tuple(child)
                (chunk_positions, valid_positions) = place_chunks(child_job_starting_positions, [child_job_directions], chunk_job, chunk_dep_iteration, floor_size, robot_starting_positions)
                if valid_positions == True:
                    try:
                        print_direction = chunk_print_direction(child_job_directions, chunk_job)     
                        (total_print_time, path_error) = schedule(robot_starting_positions, floor_size, chunk_dep_iteration, chunk_job, chunk_print_time, chunk_positions, print_direction)
                        if path_error == True:
                            valid_positions = False
                    except IndexError:
                        valid_positions = False
                    except UnboundLocalError:
                        valid_positions = False
                        
                iteration_count += 1
            
            #if child does not form in 100 iterations, pick parent one to move to next generation
            if iteration_count >= 100:
                parent_index = select_parent(probabilites)
                iteration_tuple = sorted_population_tuple[parent_index]
                
                #FOR TESTING
                # print("Iteration count exceeded, carry " + str(p1_index) + " to next generation")
                # print("Mutation is " + str(mutation))
                # print(total_print_time)
            else:
                iteration_tuple = (child_job_starting_positions, array(child_job_directions), total_print_time)
                
                #FOR TESTING
                # print("Parents: " + str(p1_index) + " and " + str(p2_index))
                # print("Mutation is " + str(mutation))
                # print(total_print_time)
    
            #add to new population once validity is confirmed
            new_population.append(iteration_tuple)
            print_time_pop[num_elite+individual] = total_print_time
        
        for individual in range(0,int(num_new_random)):
            (total_print_time, job_starting_posiitons, job_directions) = create_random_configuration(floor_size, chunk_dependencies, chunk_job, robot_starting_positions, chunk_print_time)
            
            # print(total_print_time)
            
            #evaluate results    
            iteration_tuple = (job_starting_posiitons, job_directions, total_print_time)
            new_population.append(iteration_tuple)
            print_time_pop[num_elite+num_crossover+individual] = total_print_time
        
        print_time_pop.sort()
        new_population = sorted(new_population, key = lambda individual: individual[2])
        
        sorted_population_tuple = deepcopy(new_population)
    
        #FOR TESTING
        print("Generation " +str(generation) +" population print time: \n" + str(print_time_pop))
        
        #Write print and configuration results
        with open(complete_filename, "a+") as file:
           file.write("Generation " +str(generation) +" population print time: \n" + str(print_time_pop) + "\n")
           
        with open(config_complete_filename, "a+") as file:
            file.write("Generation " +str(generation) +" population configuration: \n" + str(new_population) + "\n")
            
        generation += 1
        
        convergence = convergence_test(print_time_pop, num_pop, percent_random)
    
    if convergence == True:
        print("Converged in " + str(generation-1) + " generations!")
        with open(config_complete_filename, "a+") as file:
            file.write("Converged in " + str(generation) + " generations!")