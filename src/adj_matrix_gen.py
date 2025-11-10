#!/usr/bin/env python3

import numpy as np
import csv

"""
Takes a unit adjacency txt and a series of waypoint coordinates and returns an adjacency matrix .npy
where each row represents a node, and its adjacent nodes are weighted with the euclidian distance (0 if unconnected).
"""

def main():
    waypoints = np.loadtxt("../waypoints/sebstest.csv", delimiter=',')
    
    with open("../waypoints/sebstest_unit_adj.csv") as file:
        unit_adj = list()
        for line in file:            
            unit_adj.append(np.fromstring(line, sep=',', dtype=int))
    
    adj_matrix = np.zeros((len(waypoints),len(waypoints)))
    for node, item in enumerate(unit_adj):
        node_coordinates = np.array([ waypoints[node][0] , waypoints[node][1] ])
        for neighbour in item:
            neighbour_coordinates = np.array([ waypoints[neighbour][0] , waypoints[neighbour][1] ])
            adj_matrix[node][neighbour] = np.linalg.norm(node_coordinates - neighbour_coordinates)
    
    # for count, row in enumerate(unit_adj):
    #     print(row)
    #     print(adj_matrix[count])
        
    np.save("../waypoints/sebstest_adj_matrix.npy", adj_matrix)

if __name__ == '__main__':
    main()