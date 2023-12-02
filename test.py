# import numpy as np
# import random

# distance_matrix = np.array([[0,12,20,25,30,30],
#                             [12,0,10,11,22,30],
#                             [20,10,0,2,11,25],
#                             [25,11,2,0,10,20],
#                             [30,22,11,10,0,12],
#                             [30,30,25,20,12,0]], dtype=int)

# pheromone_matrix = np.ones_like(distance_matrix) * 0.5  # Initial pheromone matrix

# current_city = np.random.randint(5)
# visited_cities = [current_city]

# probabilities = (pheromone_matrix[current_city]) * ((1 / distance_matrix[current_city]))
# probabilities /= probabilities.sum()
# random = random.random()

# probabilities[visited_cities] = 0  # Exclude visited cities
# print(probabilities)
# print(pheromone_matrix)

print([(1,2,3), (4,5,6)] == [(1,2,3), (3,5,6)])