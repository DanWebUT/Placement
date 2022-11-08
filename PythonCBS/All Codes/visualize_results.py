"""
Code to visualize results
"""
from placer import place_chunks
from placement_visualizer import placement_vis
from numpy import array
from scheduler import schedule



#input starting chunks configuration
# job_starting_posiitons = array([[1,1],
#        [ 5, 1],
#        [ 1,  5],
#        [ 5,  5]])
#input starting chunks direction
# job_directions = [[2, 2, 2, 2]]

job_starting_posiitons = array([[2, 3],
       [4, 3],
       [2, 5],
       [8, 2]])
job_directions = [[0, 1, 2, 1]]

#other inputs
#Tall Job
chunk_dependencies = [[],[0],[],[2],[],[4],[],[6],\
                      [0],[1,8],[2],[3,10],[4],[5,12],[6],[7,14],\
                      [8],[9,16],[10],[11,18],[12],[13,20],[14],[15,22]]
chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]

# #rotated tall box
# chunk_dependencies = [[],[0, 2],[],[],[3, 5],[],[],[6, 8], [],[],[9, 11],[],\
#                       [0],[1,12,14],[2],[3],[4,15,17],[5],[6],[7,18,20],[8],[9],[10,21,23],[11]]
    
# chunk_job = [[0],[0],[0],[1],[1],[1],[2],[2],[2],[3],[3],[3],[0],[0],[0],[1],[1],[1],[2],[2],[2],[3],[3],[3]]

# #Pyramid
# chunk_dependencies = [[],[0,2],[],[],[3,5],[],[],[6],[], \
#                       [0],[1,7,11],[2],[3],[4,12,14],[5],[6],[7,15],[8], \
#                       [9],[10,18,20],[11],[12],[13,21,23],[14],[15],[16,24], \
#                       [18],[19,26,28],[20],[21],[22,29,31],[23], \
#                       [26],[27,32,34],[28]]
# chunk_job = [[0],[0],[0],[1],[1],[1],[2],[2],[3],\
#               [0],[0],[0],[1],[1],[1],[2],[2],[3],\
#               [0],[0],[0],[1],[1],[1],[2],[2], \
#               [0],[0],[0],[1],[1],[1],\
#               [0],[0],[0]]

# #Resized Benchy
# chunk_dependencies = [[],[0],[],[2],[],[4],[],[6],\
#                       [0],[1,8],[2],[3,10],[4],[5,12],\
#                       [8],[9,14],[10],[11,16]]
# chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[0],[0],[1],[1]]


robot_starting_positions = [[0,0],[1,0],[2,0],[3,0]]
floor_size = [8,6]

#create other chunks
(chunk_positions, valid_positions) = place_chunks(job_starting_posiitons, job_directions, chunk_job, chunk_dependencies, floor_size, robot_starting_positions)

#visualize placement
placement_vis(floor_size, chunk_positions, chunk_job)

