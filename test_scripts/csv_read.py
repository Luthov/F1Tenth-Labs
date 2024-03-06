import csv
import numpy as np

f_in = open("waypoint_data.csv", newline="")

reader = csv.reader(f_in)

# Create a numpy array to store the data    
waypoints = np.zeros((0, 2))

for row in reader:
    # Append the row to the numpy array
    waypoints = np.vstack([waypoints, [row[0], row[1]]])

print(waypoints[0])

f_in.close()