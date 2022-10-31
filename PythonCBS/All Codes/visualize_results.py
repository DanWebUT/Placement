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
job_starting_posiitons = array([[ 2,  4],[ 6,  3],[ 6,  5],[12,  0]])
job_directions = [[0, 0, 2, 2]]

#other inputs
#Tall Job
chunk_dependencies = [[],[0],[],[2],[],[4],[],[6],\
                      [0],[1,8],[2],[3,10],[4],[5,12],[6],[7,14],\
                      [8],[9,16],[10],[11,18],[12],[13,20],[14],[15,22]]
chunk_job = [[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3],[0],[0],[1],[1],[2],[2],[3],[3]]
robot_starting_positions = [[0,0],[1,0],[2,0],[3,0]]
floor_size = [8,6]

#create other chunks
(chunk_positions, valid_positions) = place_chunks(job_starting_posiitons, job_directions, chunk_job, chunk_dependencies, floor_size, robot_starting_positions)

#visualize placement
placement_vis(floor_size, chunk_positions, chunk_job)

