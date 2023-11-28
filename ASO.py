import numpy as np



# ACO Parameters
num_ants = 5
num_iterations = 10
initial_pheromone_level = 0.5
evaporation_rate = 0.5
alpha = 1  # Influence of pheromone
beta = 2   # Influence of distance

# Distance matrix between cities (pheromones represent attractiveness)
distance_matrix = np.zeros((5, 5), dtype=int)

pheromone_matrix = np.ones_like(distance_matrix) * initial_pheromone_level  # Initial pheromone matrix

# Ant Colony Optimization algorithm
def ant_colony_optimization():

    for iteration in range(num_iterations):
        ant_paths = []


        for ant in range(num_ants):
            current_city = np.random.randint(len(distance_matrix))
            visited_cities = [current_city]

            while len(visited_cities) < len(distance_matrix):
                # Calculate probabilities for next city selection
                probabilities = (pheromone_matrix[current_city] ** alpha) * ((1 / distance_matrix[current_city]) ** beta)
                probabilities[visited_cities] = 0  # Exclude visited cities

                # Normalize probabilities
                probabilities /= probabilities.sum()

                # Select next city based on probabilities
                # Transition rule: Roulette wheel selection
                next_city = np.random.choice(np.arange(len(distance_matrix)), p=probabilities)
                visited_cities.append(next_city)
                current_city = next_city

            ant_paths.append(visited_cities)

        # Update pheromone levels
        # Evaporation
        pheromone_matrix *= (1 - evaporation_rate)  # Evaporation
        
        # Pheromone deposition
        for path in ant_paths:
            for i in range(len(path) - 1):
                pheromone_matrix[path[i], path[i + 1]] += 1 / distance_matrix[path[i], path[i + 1]]

# After iterations, use the pheromone trails to find the best path
best_path = np.argmax(pheromone_matrix)
print("Best path:", best_path)
