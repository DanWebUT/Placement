import numpy as np
"""
This method straightens all paths and calculates their lengths. It also generates
a visual path which is compatible with the visualizer method
"""

def scrub_paths(paths):
    robot_paths = []
    robot_path_lengths = np.zeros(len(paths))
    
    rounded_paths = np.around(paths, -2)
    
    #New method: just round everything to the nearest 100 and remove everything in between.
    for robot, robot_path in enumerate(rounded_paths):
        new_path = []
        path_length = len(robot_path)
        for count, point in enumerate(robot_path):
            if count != path_length-1:
                next_point = robot_path[count+1]
                if np.array_equal(point, next_point) != True:
                    new_path.append(point)
        new_path = np.array(new_path)
        robot_paths.append(new_path)
        print(new_path)
        robot_path_lengths[robot] = len(new_path)
    
    # #correct paths to disallow diagonals
    # for robot_path in paths:
    #     for count, point in enumerate(robot_path):
    #         if count == 0:
    #             pass
    #         elif count == len(robot_path)-1:
    #             pass
    #         else:
    #             prev_point = robot_path[count-1]
    #             next_point = robot_path[count+1]
    #             #get rid of stuttering
    #             #check X
    #             if prev_point[0] == next_point[0] and prev_point[0] != point[0]:
    #                 point[0] = prev_point[0]
    #             #check Y
    #             if prev_point[1] == next_point[1] and prev_point[1] != point[1]:
    #                 point[1] = prev_point[1]
    
    
    # # get rid of diags
    # for robot, robot_path in enumerate(paths):
    #     new_path = []
    #     skip = 0
    #     end_point = robot_path[len(robot_path)-1]
    #     for count, point in enumerate(robot_path):
    #         if skip == 0:
    #             if count == 0:
    #                 new_path.append(point)
    #             elif count == len(robot_path)-1:
    #                 new_path.append(point)
    #             elif point[0] == end_point[0] and point[1] == end_point[1]:
    #                 pass
    #             else:
    #                 prev_point = robot_path[count-1]
    #                 next_point = robot_path[count+1]
    #                 new_path.append(point)
    #                 if point[0] != next_point[0] and next_point[1] != point[1]:
    #                     #find next point where line is straight
    #                     test_point_1 = point
    #                     test_point_2 = next_point
    #                     diag_points_counter = 0
    #                     #find the direction of previous move 0 for x, 1 for y
    #                     if prev_point[0] == point [0]:
    #                         direction = 0
    #                     elif prev_point[1] == point [1]:
    #                         direction = 1
                        
    #                     diag_points_counter = 0
    #                     while test_point_1[0] != test_point_2[0] and test_point_1[1] != test_point_2[1]:
    #                         diag_points_counter = diag_points_counter + 1
    #                         test_point_1 = test_point_2
    #                         test_point_2 = robot_path[count+1+diag_points_counter]
                        
    #                     skip = diag_points_counter + 1
                        
    #                     for i in range(1,diag_points_counter+1):
    #                         if direction == 0:
    #                             new_point = [point[0],robot_path[count+i][1]]
    #                         if direction == 1:
    #                             new_point = [robot_path[count+i][0],point[1]]
    #                         new_path.append(np.array(new_point))
                                
    #                     for i in range(1, diag_points_counter+2):
    #                         if direction == 0:
    #                             new_point = [robot_path[count+i][0],robot_path[count+diag_points_counter][1]]
    #                         if direction == 1:
    #                             new_point = [robot_path[count+diag_points_counter][0],robot_path[count+i][1]]
    #                         new_path.append(np.array(new_point))
    #         else:
    #             skip = skip -1
    #     new_path = np.array(new_path)                   
    #     robot_paths.append(new_path)
    #     robot_path_lengths[robot] = len(new_path)
    
    #visualize paths
    robot_visualize_paths = []
    longest_path = int(max(robot_path_lengths))
    for i in range(0, len(robot_paths)):
        visualize_path = []
        for j in range(0,longest_path):
            if j < int(robot_path_lengths[i]):
                new_point = [robot_paths[i][j][0],robot_paths[i][j][1]]
            else:
                new_point = [robot_paths[i][int(robot_path_lengths[i]-1)][0],robot_paths[i][int(robot_path_lengths[i]-1)][1]]
            visualize_path.append(new_point)
        visualize_path = np.array(visualize_path)
        robot_visualize_paths.append(visualize_path)
    robot_visualize_paths = np.array(robot_visualize_paths)
    
    return(robot_path_lengths,robot_paths,robot_visualize_paths)
            
            