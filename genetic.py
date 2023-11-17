import random
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


from objective_function import calculate_total_fitness, generate_initial_solution,check_feasibility, build_grid, tweak_path, tweak_path_crossover

#Genetic Algorithm Parameters
num_generations = 100
population_size  = 10
p_elite = 0.2
p_cross = 0.6
p_mutation = 0.2
# alpha = 0.7


num_crossover = int(p_cross * population_size)
num_mutation = int(p_mutation * population_size)
num_elite = int(p_elite * population_size)



# def crossover_one_point(parent1, parent2, crossover_point):
#     child1 = np.concatenate((parent1[:crossover_point], parent2[crossover_point:]))
#     child2 = np.concatenate((parent2[:crossover_point], parent1[crossover_point:]))
    
#     return child1, child2


# def crossover_multi_point(parent1, parent2, crossover_point1, crossover_point2):
#     child1 = np.concatenate((parent1[:crossover_point1], parent2[crossover_point1:crossover_point2], parent1[crossover_point2:]))
#     child2 = np.concatenate((parent2[:crossover_point1], parent1[crossover_point1:crossover_point2], parent2[crossover_point2:]))

#     return child1, child2


# def crossover_uniform(parent1, parent2, crossover_prob):
#     mask = np.random.rand(len(parent1)) < crossover_prob
#     child1 = np.where(mask, parent1, parent2)
#     child2 = np.where(mask, parent2, parent1)

#     return child1, child2


def crossover(parent1, parent2, grid, drone_occ_1, drone_occ_2):
    remaining = [i for i in range(len(parent1) - 1)]
    child1 = None
    child2 = None
    while len(remaining) > 0:
        x = random.choice(remaining)
        picked_x = remaining[x]
        remaining.remove(picked_x)
        child1,child2,drone_occ_new_1,drone_occ_new_2 = tweak_path_crossover(parent1, parent2, picked_x, drone_occ_1, drone_occ_2, grid)
        if(len(child1)!= 0 and len(child2)!=0):
            return child1,child2,drone_occ_new_1,drone_occ_new_2
    else:
        return parent1,parent2, drone_occ_1,drone_occ_2



        
    




def mutate(gene, drone_occupancy, grid):
    random_r1 = random.randint(0, len(gene) - 1)
    test_tweak = tweak_path(gene, random_r1, drone_occupancy, gene[random_r1][0], gene[random_r1][-1], grid)
    if len(test_tweak) == 0:
        return gene, drone_occupancy
    else:
        return test_tweak


def select_elite(population, fitness):
    elite_indices = np.argsort(fitness)[num_elite:]

    return [population[i] for i in elite_indices]

def select_worst(population, fitness):
    worst_indices = np.argsort(fitness)[-1 * num_crossover:]

    return [population[index] for index in worst_indices],worst_indices




def fitness_proportionate_selection(population, fitness_scores, num_selected):
    # Calculate total fitness
    total_fitness = sum(fitness_scores)
    # Calculate selection probabilities
    selection_probabilities = [score / total_fitness for score in fitness_scores]
    # Select individuals based on probabilities
    selected_indices = []
    for _ in range(num_selected):
        rand_num = random.random()
        cumulative_prob = 0
        # Iterate through individuals and check if they are selected
        for i, prob in enumerate(selection_probabilities):
            cumulative_prob += prob
            if rand_num <= cumulative_prob:
                selected_indices.append(i)
                break

    # Return the selected individuals
    selected_population = [population[i] for i in selected_indices]
    return selected_population, selected_indices


def generate_population(size_of_grid, starting_points, target_points, obstacles):
    population = []
    drone_occupancies = []
    grid = []
    for i in range(population_size):
        paths,grid, drone_occupancy = generate_initial_solution(size_of_grid, starting_points, target_points, obstacles)
        population.append(paths)
        drone_occupancies.append(drone_occupancy)
    return population, drone_occupancies,grid

def genetic(size_of_grid, starting_points, target_points, obstacles):
    population, drone_occupancies, grid = generate_population(size_of_grid, starting_points, target_points, obstacles)
    for iteration in range(num_generations):
        fitness = []
        for gene in population:
            fitness.append(calculate_total_fitness(gene))
        new_population = []
        elites = select_elite(population, fitness)
        for elite in elites:
            new_population.append(elite)
        
        parents, indices = fitness_proportionate_selection(population, fitness, num_crossover)

        for i in range(0, len(parents), 2):
            # Select parents
            parent1 = parents[i]
            parent2 = parents[i + 1]
            # Perform crossover
            print(drone_occupancies[indices[i]])
            children = crossover(parent1, parent2, grid, drone_occupancies[indices[i]], drone_occupancies[indices[i + 1]])
            new_population.append(child for child in children)
        # Perform mutation
        worst_genes, worst_gene_indices = select_worst(population, fitness)
        for i,gene in enumerate(worst_genes):
            new_population.append(mutate(gene,drone_occupancies[worst_gene_indices[i]],grid))
    
        # Output best solution and fitness
        best_index = np.argmax(fitness)
        best_solution = population[best_index]
        best_fitness = fitness[best_index]
        print(f"Iteration {iteration + 1}: Best Fitness = {best_fitness}")

    return best_solution, best_fitness,population

size_of_grid1 = 30  # Size of the grid

# Define start points for the drones (x, y, z)
ps_list1 = [(5, 5, 5), (1, 10, 10), (20, 20, 20)]

# Define target points for the drones (x, y, z)
pt_list1 = [(25, 25, 25), (1, 15, 20), (18, 12, 12)]

# Define obstacles [(x, y, z) (x, y, z)] all grid cells from x1 to x2 and y1 to y2 and z1 to z2 are obstacles
obstacle_list = [[(2, 1, 1), (3, 2, 6)], [(2, 3, 1), (3, 6, 6)]]
obstacle_list1 = [[(8, 8, 8), (12, 12, 12)], [(20, 15, 10), (25, 18, 20)], [(7, 15, 12), (10, 20, 18)]]

best_solution, best_fitness,population = genetic(size_of_grid1, ps_list1, pt_list1, obstacle_list1)
