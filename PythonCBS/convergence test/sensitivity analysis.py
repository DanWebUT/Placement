from placer import placement_optimization

import csv

chunk_dependencies = [[],[0, 2],[],[],[3, 5],[],[],[6, 8], [],[],[9, 11],[],\
                      [0],[1,12,14],[2],[3],[4,15,17],[5],[6],[7,18,20],[8],[9],[10,21,23],[11]]
    
chunk_job = [[0],[0],[0],[1],[1],[1],[2],[2],[2],[3],[3],[3],[0],[0],[0],[1],[1],[1],[2],[2],[2],[3],[3],[3]]

chunk_print_time = [2253., 2859., 1552., 2253., 2859., 1552., 2253., 2859., 1552., 2253., 2859., 1552., \
        1894, 2598, 1194, 1894, 2598, 1194, 1894, 2598, 1194, 1894, 2598, 1194]
robot_starting_positions = [[0,0],[1,0],[2,0],[3,0]]
floor_size = [8,6]


chance_mutation = 0
mutation_step = .1
chance_crossover = 0
crossover_step = .1
# num_generations = 100
percent_elite = 0
elite_step = .1
percent_random = .2 #percent of new randomly generated populateion
random_step = .1

header = ['Chance Mutation', 'Chance Crossover', 'Percent Elite', 'Percent Random','Number of Generations','Best Value']

with open("Sensitivity Analysis Results.csv", "w") as file:
    writer = csv.writer(file)
    writer.writerow(header)
    while percent_random <= .4:
        percent_elite = 0
        while percent_elite <= .3:
            chance_crossover = 0
            while chance_crossover <= 1:
                chance_mutation = 0
                while chance_mutation <= 1:
                    (generation, best_value) = placement_optimization(chance_mutation, chance_crossover, percent_elite, percent_random, chunk_dependencies, chunk_job, chunk_print_time, robot_starting_positions, floor_size)
                    print("Generation: " + str(generation)+ ", Best Value: " + str(best_value))
                    
                    data = [chance_mutation, chance_crossover, percent_elite, percent_random, generation,best_value]
                    writer.writerow(data)
                    
                    chance_mutation += mutation_step
                chance_crossover += crossover_step
            percent_elite += elite_step
        percent_random += random_step
                    
                    
                    
        