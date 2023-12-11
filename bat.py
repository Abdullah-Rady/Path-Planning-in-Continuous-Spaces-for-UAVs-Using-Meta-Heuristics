import numpy as np
import time
import random
from objective_function import calculate_total_fitness, generate_initial_solution, tweak_path_cross
from visualize import calculate_stats, plot_best_fitness_over_iterations, plot_fitness_over_iterations, save_scenario_stats_to_json, visualize_problem_solution, get_paths

# bat algorithm implementation based on https://arxiv.org/abs/1004.4170

max_iterations = 100
num_of_bats = 10
lower_bound = 0
upper_bound = 1
loudness = 0
pulse_rate = 0
alpha = 0.5
gamma = 0.9

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



def bat_optimization(size_of_grid, starting_points, target_points, obstacles, visualize=False):

    population, drone_occupancies, grid = generate_bats(size_of_grid, starting_points, target_points, obstacles)

    bat_velocity = [
        [
            [
                (0, 0, 0) for _ in range(len(population[i][j]))
            ] for j in range(len(population[i]))
        ] for i in range(num_of_bats)
    ]

    # Initialize global best position and score
    bat_best_pos = population.copy()
    bat_best_score = np.full(num_of_bats, np.inf)


    # Initialize global best position and score
    global_best_bat = []
    global_best_score = np.inf

    # print("global best pos ",global_best_position)

    for _ in range(max_iterations):

        all_fitness = []

        for i in range(num_of_bats):

            score = calculate_total_fitness(population[i])

            all_fitness.append(score)

            if score < bat_best_score[i]:
                bat_best_pos[i] = population[i]
                bat_best_score[i] = score

            if visualize:
                print(f" score: {score}")


            if score < global_best_score:
                global_best_score = score
                global_best_bat = population[i]

        if visualize:
            print(f"best score: {global_best_score}")

        for i in range(num_of_bats):
        

            new_drone_occupancy = [[[ [] for _ in range(len(grid))] for _ in range(len(grid))] for _ in range(len(grid))]            

            for j in range(len(population[i])):

                #update velocity
                frequency = np.random.uniform(0, 1, 1)               
                bat_velocity[i][j] = [[(a1 - global_best_bat[j][ii][0]) * frequency, (b1 + global_best_bat[j][ii][2]) * frequency, (c1 + global_best_bat[j][ii][2]) * frequency] for (ii, (a1, b1, c1)) in  enumerate(bat_velocity[i][j]) ]
                

                #update position
                old_population = population[i][j].copy()
                population[i][j] = [[(int(round(a1 + a2))), (int(round(b1 + b2))), (int(round(c1 + c2)))] for (a1, b1, c1), (a2, b2, c2) in zip(population[i][j], bat_velocity[i][j])]
                
                if population[i][j] == old_population:
                    continue
                
                # print("particle " + str(i) + " tweaking path:" + str(j))
                new_path, drone_occupancy_copy = tweak_path_cross(population[i], j, population[i][j], new_drone_occupancy, starting_points[j], target_points[j], grid)
                # print("finished tweaking path")
        
                if len(new_path) == 0:
                    # print("couldnt find a new path")
                    population[i][j] = old_population
                    get_old_occupancies(drone_occupancies[i], new_drone_occupancy, j)
                    continue

            
                new_drone_occupancy = drone_occupancy_copy
                population[i][j] = new_path

                if np.random.uniform(0, 1, 1) > pulse_rate:
                    random_path = random.randint(0, len(global_best_bat) - 1)
                    new_solution = [ (a + random.randint(-size_of_grid/4, size_of_grid/4) * loudness, b + random.randint(-size_of_grid/4, size_of_grid/4) * loudness, c + random.randint(-size_of_grid/4, size_of_grid/4) * loudness) for (a, b, c) in global_best_bat[random_path]]

                tweak_path_cross(new_solution, random_path, new_solution[random_path], new_drone_occupancy, starting_points[j], target_points[j], grid)
                new_fitness = calculate_total_fitness(new_solution)

                if len(new_path) == 0:
                    continue

                new_drone_occupancy = drone_occupancy_copy

                if np.random.uniform(0, 1, 1) < alpha and new_fitness < bat_best_score[i]:
                    bat_best_pos[i][j] = new_solution
                    bat_best_score[i][j] = new_fitness
                
                if new_fitness < global_best_score:
                    global_best_bat = new_solution
                    global_best_score = new_fitness


            drone_occupancies[i] = new_drone_occupancy


            

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
best_position, best_score, all_fitness = bat_optimization(size_of_grid1, ps_list1, pt_list1, obstacle_list1, visualize=True)
end_time = time.time()

print(calculate_stats(all_fitness, start_time,end_time))

plot_fitness_over_iterations(all_fitness)
plot_best_fitness_over_iterations(all_fitness)

visualize_problem_solution(ps_list2, pt_list2, obstacle_list2, best_position)
# print(f"Best selected indices: {np.nonzero(best_position)[0]}")
# print(f"Best score: {best_score}")
