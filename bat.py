import numpy as np
import time
import random
from objective_function import calculate_total_fitness, generate_initial_solution, tweak_path_cross, calculate_single_path_fitness
from visualize import calculate_stats, plot_best_fitness_over_iterations, plot_fitness_over_iterations, save_scenario_stats_to_json, visualize_problem_solution, get_paths

# bat algorithm implementation based on https://arxiv.org/abs/1004.4170

max_iterations = 100
num_of_bats = 10
lower_bound = 0
upper_bound = 1
alpha = 0.5
gamma = 0.9
min_frequency = 0
max_frequency = 2
min_loudness = 0
max_loudness = 3
min_pulse_rate = 0
max_pulse_rate = 1

def generate_bats(size_of_grid, starting_points, target_points, obstacles):
    population = []
    drone_occupancies = []
    grid = []

    for _ in range(num_of_bats):
        paths, grid, drone_occupancy = generate_initial_solution(size_of_grid, starting_points, target_points, obstacles)
        population.append(paths)
        drone_occupancies.append(drone_occupancy)

    return population, drone_occupancies,grid

def get_old_occupancies(old_drone_occupancy, new_drone_occupancy, pos):
           for n in range(len(old_drone_occupancy)):
                        for m in range(len(old_drone_occupancy[n])):
                            for k in range(len(old_drone_occupancy[n][m])):
                               for s in range(len(old_drone_occupancy[n][m][k])):
                                    if old_drone_occupancy[n][m][k][s][0] == pos + 1:
                                        new_drone_occupancy[n][m][k].append(old_drone_occupancy[n][m][k][s])


def bat_optimization(size_of_grid, starting_points, target_points, obstacles, visualize):

    population, drone_occupancies, grid = generate_bats(size_of_grid, starting_points, target_points, obstacles)

    bat_velocity = [
        [
            [
                (0, 0, 0) for _ in range(len(population[i][j]))
            ] for j in range(len(population[i]))
        ] for i in range(num_of_bats)
    ]

    global_best_score = np.inf
    loudness = [ np.random.uniform(min_loudness,max_loudness,1) for _ in range(num_of_bats)]
    initial_pulse_rate = [ np.random.uniform(min_pulse_rate,max_pulse_rate,1) for _ in range(num_of_bats)]
    pulse_rate = initial_pulse_rate.copy()
    all_fitness = []
    for t in range(max_iterations):
        current_fitness = []
        print(f"Iteration: {t}")
        for i in range(num_of_bats):
            fitness = calculate_total_fitness(population[i])
            if visualize:
                print(f"Bat: {i} Fitness: {fitness}")
                print(f"Solution: {population[i]}")
            all_fitness.append(fitness)
            current_fitness.append(fitness)
            if fitness < global_best_score:
                global_best_score = fitness
                global_best_bat = population[i]
        # print(f"Current fitnesses: {current_fitness}")
        for i in range(num_of_bats):
            if(population[i] == global_best_bat):
                continue
            if visualize:
                print(f"Bat: {i}")
            new_drone_occupancy = [[[ [] for _ in range(len(grid))] for _ in range(len(grid))] for _ in range(len(grid))]            
            for j in range(len(population[i])):
                if visualize:
                    print(f"Drone: {j+1}")
                frequency = np.random.uniform(0, 1, 3)
                min_length = min(len(bat_velocity[i][j]), len(global_best_bat[j]))     
                bat_velocity[i][j] = [ [ (a1 - global_best_bat[j][ii][0]) * frequency[0], (b1 + global_best_bat[j][ii][1]) * frequency[1], (c1 + global_best_bat[j][ii][2]) * frequency[2] ] for (ii,(a1, b1, c1)) in  enumerate(bat_velocity[i][j][:min_length]) ]
                new_path_before_check = [[(int(np.round(a1 + a2))), (int(np.round(b1 + b2))), (int(np.round(c1 + c2)))] for (a1, b1, c1), (a2, b2, c2) in zip(population[i][j], bat_velocity[i][j])]
                if visualize:
                    print("Tweaking path")
                new_path, drone_occupancy_copy = tweak_path_cross(population[i], j, new_path_before_check, new_drone_occupancy, starting_points[j], target_points[j], grid,visualize=visualize)
                if len(new_path) == 0:
                    if visualize:
                        print("No path found")
                    get_old_occupancies(drone_occupancies[i], new_drone_occupancy, j)
                    continue
                new_drone_occupancy = drone_occupancy_copy
                population[i][j] = new_path
                if visualize:
                    print("Path tweaked")
            drone_occupancies[i] = new_drone_occupancy

            rand1 = np.random.uniform(0, 1, 1)
            rand2 = np.random.uniform(0, 1, 1)
            new_member = []
            new_member_drone_occupancy = [[[ [] for _ in range(len(grid))] for _ in range(len(grid))] for _ in range(len(grid))]            
            for j in range(len(population[i])):
                if rand1 > pulse_rate[i]:
                    if visualize:
                        print("Searching around best")
                    avg_loudness = np.mean(loudness)
                    rand = np.random.uniform(-1, 1, 1)
                    new_solution_around_best = [(int(np.round(a + rand * avg_loudness)),int(np.round(b + rand * avg_loudness)),
                        int(np.round(c + rand * avg_loudness))) for (a, b, c) in global_best_bat[j]]
                    if visualize:
                        print(f"Tweaking path: {new_solution_around_best} around best", )                    
                    tweaked_solution, drone_occupancy_copy = tweak_path_cross(population[i], j, new_solution_around_best, new_member_drone_occupancy, starting_points[j], target_points[j], grid, visualize=visualize)
                    if len(tweaked_solution) == 0:
                        if visualize:
                            print("No path around best found")
                        break
                    if visualize:
                        print("Path found around best")
                    new_member.append(tweaked_solution)
                    new_member_drone_occupancy = drone_occupancy_copy
            if(len(new_member) == len(starting_points)):
                new_fitness = calculate_total_fitness(new_member)
                if new_fitness < current_fitness[i] and rand2 < loudness[i]:
                    population[i] = new_member
                    drone_occupancies[i] = new_member_drone_occupancy
                    loudness[i] = alpha * loudness[i]
                    pulse_rate[i] = initial_pulse_rate[i] * (1 - np.exp(-gamma * t))
    return global_best_bat, global_best_score, all_fitness


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
start_time = time.time()
best_position, best_score, all_fitness = bat_optimization(size_of_grid2, ps_list2, pt_list2, obstacle_list2 ,visualize=False)
end_time = time.time()

print(calculate_stats(all_fitness, start_time,end_time))

plot_fitness_over_iterations(all_fitness)
plot_best_fitness_over_iterations(all_fitness)

visualize_problem_solution(ps_list2, pt_list2, obstacle_list2, best_position)
# print(f"Best selected indices: {np.nonzero(best_position)[0]}")
# print(f"Best score: {best_score}")
