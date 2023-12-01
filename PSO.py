import numpy as np
import time
from objective_function import calculate_total_fitness, generate_initial_solution
from visualize import calculate_stats, plot_best_fitness_over_iterations, plot_fitness_over_iterations, save_scenario_stats_to_json, visualize_problem_solution, get_paths

# Parameters
# num_elements = 7  # Number of elements or items to select from
swarm_size = 10
max_iterations = 100
inertia_weight = 0.5
c1 = 1.5
c2 = 1.5


def generate_swarm(size_of_grid, starting_points, target_points, obstacles):
    population = []
    drone_occupancies = []
    grid = []

    for _ in range(swarm_size):
        paths,grid, drone_occupancy = generate_initial_solution(size_of_grid, starting_points, target_points, obstacles)
        population.append(paths)
        drone_occupancies.append(drone_occupancy)

    return population, drone_occupancies,grid

def particle_swarm_optimization(objective_function, size_of_grid, starting_points, target_points, obstacles, visualize=False):
    # Initialize particles randomly selecting indices within the range of elements
    population, drone_occupancies, grid = generate_swarm(size_of_grid, starting_points, target_points, obstacles)
    num_elements = len(population[0])

    # Initialize velocities as binary values
    particles_velocity = np.random.uniform(-1, 1, size=(swarm_size, num_elements))

    # Initialize best known positions and scores for each particle
    personal_best_position = population.copy()
    personal_best_score = np.full(swarm_size, np.inf)

    # Initialize global best position and score
    global_best_position = np.zeros(num_elements)
    global_best_score = np.inf

    for _ in range(max_iterations):

        for i in range(swarm_size):

            score = calculate_total_fitness(population[i])
            if score < personal_best_score[i]:
                personal_best_score[i] = score
                personal_best_position[i] = population[i]

            if score < global_best_score:
                global_best_score = score
                global_best_position = population[i]

        for i in range(swarm_size):
            inertia = inertia_weight * particles_velocity[i]
            cognitive = c1 * np.random.random() * (personal_best_position[i] - population[i])
            social = c2 * np.random.random() * (global_best_position - population[i])
            particles_velocity[i] = inertia + cognitive + social

            # Apply velocity updates (binary, where 0 means element not selected and 1 means selected)
            population[i] = np.clip(population[i] + particles_velocity[i], 0, 1)

    return global_best_position, global_best_score


size_of_grid1 = 30  # Size of the grid
size_of_grid2 = 50  # Size of the grid


# Define start points for the drones (x, y, z)
ps_list1 = [(5, 5, 5), (1, 10, 10), (20, 20, 20)]

# Define target points for the drones (x, y, z)
pt_list1 = [(25, 25, 25), (1, 15, 20), (18, 12, 12)]

# Define obstacles [(x, y, z) (x, y, z)] all grid cells from x1 to x2 and y1 to y2 and z1 to z2 are obstacles
obstacle_list1 = [[(8, 8, 8), (12, 12, 12)], [(20, 15, 10), (25, 18, 20)], [(7, 15, 12), (10, 20, 18)]]

ps_list2 = [
    (5, 5, 5),
    (1, 10, 10),
    (20, 20, 20),
    (30, 30, 30),
    (8, 15, 25),
    (12, 5, 10)
]

# Define target points for the drones (x, y, z)
pt_list2 = [
    (25, 25, 25),
    (1, 15, 20),
    (18, 12, 12),
    (35, 30, 30),
    (10, 20, 25),
    (5, 20, 5)
]

# Define obstacles [(x, y, z) (x, y, z)] all grid cells from x1 to x2 and y1 to y2 and z1 to z2 are obstacles
obstacle_list2 = [
    [(2, 1, 1), (3, 2, 6)],
    [(2, 3, 1), (3, 6, 6)],
    [(8, 8, 8), (12, 12, 12)],
    [(20, 15, 10), (25, 18, 20)],
    [(7, 15, 12), (10, 20, 18)],
]

# Run CPSO
best_position, best_score = particle_swarm_optimization_combinatorial(objective_function, num_elements, swarm_size, max_iterations, inertia_weight, c1, c2)
print(f"Best selected indices: {np.nonzero(best_position)[0]}")
print(f"Best score: {best_score}")
