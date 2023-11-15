import random
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


from objective_function import calculate_total_fitness, generate_initial_solution,check_feasibility_SA, build_grid

#Genetic Algorithm Parameters
num_generations = 200
population_size  = 5
p_elite = 0.2
p_cross = 0.6
p_mutation = 0.2
alpha = 0.7


num_crossover = p_cross * population_size
num_mutation = p_mutation * population_size
num_elite = p_elite * population_size



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


def crossover_arithmetic(parent1, parent2):
    child1 = alpha * parent1 + (1 - alpha) * parent2
    child2 = (1 - alpha) * parent1 + alpha * parent2

    return child1, child2

def mutate(gene):
    new_point = []
    new_point[0] += random.uniform(-1, 1)
    new_point[1] += random.uniform(-1, 1)
    new_point[2] += random.uniform(-1, 1)

    random_r1 = random.randint(0, len(gene))
    gene[random_r1] += new_point

    return gene


def select_elite(population, fitness):
    elite_indices = np.argsort(fitness)[num_elite:]

    return population[elite_indices]


def select_worst(population, fitness):
    worst_indices = np.argsort(fitness)[-num_crossover:]

    return population[worst_indices]

import random


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
    return selected_population



def genetic(size_of_grid, starting_points, target_points, obstacles):
    population = generate_population(chromosome_length)

    
    for iteration in range(num_generations):
        fitness = []
        for gene, i in enumerate(population):
            fitness[i] = calculate_total_fitness(gene)

        new_population = []
        elites = select_elite(population, fitness)
        new_population.append(elites)
        
        parents = fitness_proportionate_selection(population, fitness, 2*num_crossover)

        for i in range(0, len(parents), 2):
            # Select parents
            parent1 = parents[i]
            parent2 = parents[i + 1]
            # Perform crossover
            child1, child2 = crossover_arithmetic(parent1, parent2)

            # Check Feasibility

            
            # Add children to population
            new_population.append(child1)
            new_population.append(child2)
        
        for gene in select_worst(population, fitness):
            new_population.append(mutate(gene))
    

        # Output best solution and fitness
        best_index = np.argmax(fitness)
        best_solution = population[best_index]
        best_fitness = fitness[best_index]

        print(f"Iteration {iteration + 1}: Best Fitness = {best_fitness}")

    return best_solution, best_fitness