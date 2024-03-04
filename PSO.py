import numpy as np
import time
from objective_function import calculate_total_fitness, generate_initial_solution, tweak_path_cross
from visualize import calculate_stats, plot_best_fitness_over_iterations, plot_fitness_over_iterations, save_scenario_stats_to_json, visualize_problem_solution, get_paths

# Parameters
swarm_size = 10
max_iterations = 100
inertia_weight = 0.7298
c1 = 1.4960
c2 = 1.4960


def generate_swarm(size_of_grid, starting_points, target_points, obstacles):
    population = []
    drone_occupancies = []
    grid = []

    for _ in range(swarm_size):
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

def add_inertia(particles_velocity):
     for i in range(len(particles_velocity)):
                    for j in range(len(particles_velocity[i])):
                        for k in range(len(particles_velocity[i][j])):
                            particles_velocity[i][j][k] = (inertia_weight * particles_velocity[i][j][k][0], inertia_weight * particles_velocity[i][j][k][1], inertia_weight * particles_velocity[i][j][k][2])


def particle_swarm_optimization(size_of_grid, starting_points, target_points, obstacles, visualize=False):
    # Initialize particles randomly selecting indices within the range of elements
    all_fitness = []

    population, drone_occupancies, grid = generate_swarm(size_of_grid, starting_points, target_points, obstacles)
    num_elements = len(population[0])

    # Initialize velocities as binary values
    # particles_velocity = np.random.uniform(-1, 1, size=(swarm_size, num_elements))
    particles_velocity = [
        [
            [
                (0, 0, 0) for _ in range(len(population[i][j]))
            ] for j in range(len(population[i]))
        ] for i in range(swarm_size)
    ]

    # print("particles velocity ",particles_velocity)

    # Initialize best known positions and scores for each particle
    personal_best_position = population.copy()
    personal_best_score = np.full(swarm_size, np.inf)

    # print("best pos ",personal_best_position)
    # print("best score ",personal_best_score)

    # Initialize global best position and score
    global_best_position = []
    global_best_score = np.inf

    # print("global best pos ",global_best_position)

    for _ in range(max_iterations):

        particle_fitness = []
        
        print(f"Iteration: {_}")
        for i in range(swarm_size):

            score = calculate_total_fitness(population[i])
            all_fitness.append(score)
            if visualize:
                print(f" score: {score}")

            if score < personal_best_score[i]:
                personal_best_score[i] = score
                personal_best_position[i] = population[i].copy()


            if score < global_best_score:
                global_best_score = score
                global_best_position = population[i].copy()


        if visualize:
            print(f"best score: {global_best_score}")

        for i in range(swarm_size):
        

            new_drone_occupancy = [[[ [] for _ in range(len(grid))] for _ in range(len(grid))] for _ in range(len(grid))]            
            # Apply velocity updates
            for j in range(len(population[i])):

                #update velocity

                #update inertia
                add_inertia(particles_velocity)
                            
                # first constant
                t1 = c1 * np.random.random()
                # 2nd constant
                t2 = c2 * np.random.random()
             
                cognitive = [[ t1 * (a1 - a2) , t1 * (b1 - b2) , t1 * (c1 - c2)] for (a1, b1, c1), (a2, b2, c2) in zip(personal_best_position[i][j], population[i][j])]
                # cognitive = c1 * np.random.random() * (personal_best_position[i][j] - population[i][j])

                social = [[ t2 * (a1 - a2), t2 * (b1 - b2), t2 * (c1 - c2)] for (a1, b1, c1), (a2, b2, c2) in zip(global_best_position[j], population[i][j])]
                # social = c2 * np.random.random() * (global_best_position[j] - population[i][j])
               
                temp = [[(a1 + a2), (b1 + b2), (c1 + c2)] for (a1, b1, c1), (a2, b2, c2) in zip(social, cognitive)]
               
                particles_velocity[i][j] = [[(a1 + a2), (b1 + b2), (c1 + c2)] for (a1, b1, c1), (a2, b2, c2) in zip(temp, particles_velocity[i][j])]
                
                # print("particles velocity")
                # print(particles_velocity[i][j])

                #update position
                old_population = population[i][j].copy()
                population[i][j] = [[(int(round(a1 + a2))), (int(round(b1 + b2))), (int(round(c1 + c2)))] for (a1, b1, c1), (a2, b2, c2) in zip(population[i][j], particles_velocity[i][j])]
                
                # print("after update position")
                # print(population[i][j])
                
                if population[i][j] == old_population:
                    # print("particle " + str(i) + " drone " + str(j)+ " is the same as old after adding velocity")
                    continue
                
                # print("particle " + str(i) + " tweaking path:" + str(j))
                new_path, drone_occupancy_copy = tweak_path_cross(population[i], j, population[i][j], new_drone_occupancy, starting_points[j], target_points[j], grid)
                # print("finished tweaking path")
        
                if len(new_path) == 0:
                    # print("couldnt find a new path")
                    population[i][j] = old_population
                    get_old_occupancies(drone_occupancies[i], new_drone_occupancy, j)
                    continue
                
                # print("new path ", new_path)
                # print("found a new path")
                new_drone_occupancy = drone_occupancy_copy
                population[i][j] = new_path

            drone_occupancies[i] = new_drone_occupancy


            

    return global_best_position, global_best_score, all_fitness


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
best_position, best_score, all_fitness = particle_swarm_optimization(size_of_grid2, ps_list2, pt_list2, obstacle_list2, visualize=False)
end_time = time.time()

print(calculate_stats(all_fitness, start_time,end_time))

plot_fitness_over_iterations(all_fitness)
plot_best_fitness_over_iterations(all_fitness)

visualize_problem_solution(ps_list2, pt_list2, obstacle_list2, best_position)
# print(f"Best selected indices: {np.nonzero(best_position)[0]}")
# print(f"Best score: {best_score}")
