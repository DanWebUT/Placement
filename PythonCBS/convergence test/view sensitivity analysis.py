import matplotlib.pyplot as plt
import numpy as np
import csv

fig = plt.figure()
scatter3D = fig.add_subplot(projection='3d') 

with open('Sensitivity Analysis Percents Revised.csv', newline='') as file:
    reader = csv.reader(file, delimiter = ',', quotechar = '|')
    line_count = 0
    for row in reader:
        if line_count == 0:
            print(f'Column names are {", ".join(row)}')
        elif row == []:
            continue
        else:
            x = float(row[2])
            y = float(row[3])
            z = float(row[4])
            scatter3D.scatter(x,y,z)
        line_count += 1
        

scatter3D.set_xlabel('Percent Elite')
scatter3D.set_ylabel('Percent Random')
scatter3D.set_zlabel('Number Generations to convergence')

plt.show()




