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


#Global Variables
grid = None
drone_occupancy = None


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
    elite_indices = np.argsort(fitness)[-num_elite:]

    return population[elite_indices]


def genetic(size_of_grid, starting_points, target_points, obstacles):
    population = generate_population(chromosome_length)

    
    for iteration in range(num_generations):
        fitness = []
        for gene, i in enumerate(population):
            fitness[i] = calculate_total_fitness(gene)

        elite = select_elite(population, fitness)

        # Perform crossover and mutation
        offspring = []
        num_crossover = p_cross * population_size
        for _ in range(population_size - len(elite)):
            parent1, parent2 = np.random.choice(population, size=2, p=fitness/fitness.sum())
            child1, child2 = crossover_arithmetic(parent1, parent2)
            child1 = mutate(parent1, p_mutation)
            child2 = mutate(parent2, p_mutation)
            offspring.extend([child1, child2])

        population = np.vstack((elite, np.array(offspring)))

        # Output best solution and fitness
        best_index = np.argmax(fitness)
        best_solution = population[best_index]
        best_fitness = fitness[best_index]

        print(f"Iteration {iteration + 1}: Best Fitness = {best_fitness}")

    return best_solution, best_fitness