import numpy as np
from objective_function import generate_initial_solution,calculate_total_fitness, tweak_path_cross

num_iterations = 100  # Number of iterations
num_agents = 10  # Number of search agents

def generate_population(size_of_grid, starting_points, target_points, obstacles):
    population = []
    drone_occupancies = []
    grid = []

    for _ in range(num_agents):
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

def update_cat_positions(cat_population, mouse_population, cat_objective_values, starting_points, target_points, cat_drone_occupancies, grid):
    num_cats = len(cat_population)
    num_mice = len(mouse_population)

    new_cats_drone_occupancies = []
    for cat_index in range(num_cats):
        # Generate random numbers for movement
        random_value = np.random.rand()
        mouse_index = np.random.randint(num_mice)
        I = np.round(1 + np.random.rand())
        new_drone_occupancy = [[[ [] for _ in range(len(grid))] for _ in range(len(grid))] for _ in range(len(grid))]            
        new_cat = []
        for path_index, path in enumerate(cat_population[cat_index]):
            # Update cat position based on the natural behavior of cats
            output_path = []
            print("Cat Path: ", path)
            print("Mouse Path: ", mouse_population[mouse_index][path_index])
            end = min(len(path), len(mouse_population[mouse_index][path_index]))-2+1
            output_path.append(starting_points[path_index])
            for point_index in range(1,end):
                point = path[point_index]
                # Update cat position based on the chase behavior
                temp_multiplication_point = [I * coordinate for coordinate in list(point)]
                temp_mouse_point = list(mouse_population[mouse_index][path_index][point_index])
                temp_inner_result = [coordinate1 - coordinate2 for coordinate1, coordinate2 in zip(temp_multiplication_point, temp_mouse_point)]
                temp_outer_result = [random_value * coordinate for coordinate in temp_inner_result]
                temp_added_point = [coordinate1 + coordinate2 for coordinate1, coordinate2 in zip(temp_outer_result, list(point))]  
                new_cat_point = tuple(temp_added_point)
                output_path.append(new_cat_point)
            output_path.append(target_points[path_index])
            print("Old Path: ", path)
            print("New Path:", output_path)
            new_tweaked_path, new_tweaked_drone_occupancy = tweak_path_cross(cat_population[cat_index], path_index, output_path,cat_drone_occupancies[cat_index],path[0],path[-1], grid, visualize=True)
            if(len(new_tweaked_path) == 0):
                get_old_occupancies(cat_drone_occupancies[cat_index], new_drone_occupancy, path_index)
                new_cat.append(path)
                continue
            new_cat.append(new_tweaked_path)
            new_drone_occupancy = new_tweaked_drone_occupancy
        # Evaluate the objective function for the new cat position
        new_cat_objective_value = calculate_total_fitness(new_cat)

        # Update cat position if it leads to a better solution
        if new_cat_objective_value < cat_objective_values[cat_index]:
            cat_population[cat_index] = new_cat
            new_cats_drone_occupancies.append(new_drone_occupancy)
        else:
            new_cats_drone_occupancies.append(cat_drone_occupancies[cat_index])
             

    return cat_population, new_cats_drone_occupancies

def update_mouse_positions(mouse_population, havens, mouse_objective_values, starting_points, target_points, mouse_drone_occupancies, grid):
    num_mice = len(mouse_population)
    num_havens = len(havens)
    new_mice_drone_occupancies = []
    for mouse_index in range(num_mice):
        # Generate random numbers for movement
        random_value = np.random.rand()
        haven_index = np.random.randint(num_havens)
        I = np.round(1 + np.random.rand())
        new_drone_occupancy = [[[ [] for _ in range(len(grid))] for _ in range(len(grid))] for _ in range(len(grid))]            
        new_mouse = []
        for path_index, path in enumerate(mouse_population[mouse_index]):
            # Update cat position based on the natural behavior of cats
            output_path = []

            end = min(len(path), len(havens[mouse_index][path_index]))-2+1
            output_path.append(starting_points[path_index])
            for point_index in range(end):
                # Update mouse position based on the escape behavior
                temp_inner_multiplication = [I * coordinate for coordinate in list(point)]
                temp_inner_subtraction = [coordinate2 - coordinate1 for coordinate1, coordinate2 in zip(temp_inner_multiplication, list(havens[haven_index][path_index][point_index]))]
                temp_outer_multiplication = [random_value * coordinate for coordinate in temp_inner_subtraction]
                temp_mul_sign = np.sign(mouse_objective_values[mouse_index] - calculate_total_fitness(havens[haven_index]))
                temp_outer_result = [temp_mul_sign * coordinate for coordinate in temp_outer_multiplication]
                temp_added_point = [coordinate1 + coordinate2 for coordinate1, coordinate2 in zip(temp_outer_result, list(point))]
                new_mouse_point = tuple(temp_added_point)
                output_path.append(new_mouse_point)
            output_path.append(target_points[path_index])
            new_tweaked_path, new_tweaked_drone_occupancy = tweak_path_cross(mouse_population[mouse_index], path_index, output_path,mouse_drone_occupancies[mouse_index],path[0],path[-1], grid)
            if(len(new_tweaked_path) == 0):
                get_old_occupancies(mouse_drone_occupancies[mouse_index], new_drone_occupancy, path_index)
                new_mouse.append(path)
                continue
            new_mouse.append(new_tweaked_path)
            new_drone_occupancy = new_tweaked_drone_occupancy
        # Evaluate the objective function for the new cat position
        new_cat_objective_value = calculate_total_fitness(new_mouse)

        # Update cat position if it leads to a better solution
        if new_cat_objective_value < mouse_objective_values[mouse_index]:
            mouse_population[mouse_index] = new_mouse
            new_mice_drone_occupancies.append(new_drone_occupancy)
        else:
            new_mice_drone_occupancies.append(mouse_drone_occupancies[mouse_index])

    return mouse_population, new_mice_drone_occupancies

def cmbo_algorithm(grid_size, starting_points, target_points, obstacles):
    # Initialize the population matrix
    population_matrix, drone_occupancies,grid = generate_population(grid_size, starting_points, target_points, obstacles)

    fitness_values = []
    # Select populations of mice and cats
    num_cats = num_agents // 2
    num_mice = num_agents - num_cats
    for iteration in range(num_iterations):
        print("Iteration: ", iteration)
        fitness_values = []
        # Evaluate the objective function for the current population
        for element in population_matrix:
            fitness_values.append(calculate_total_fitness(element))
        print("Fitness values: ", fitness_values)
        # Sort the population based on objective function values
        sorted_indices = np.argsort(fitness_values)
        sorted_population = [population_matrix[i] for i in sorted_indices]
        sorted_drone_occupancies = [drone_occupancies[i] for i in sorted_indices]
        sorted_objective_values = [fitness_values[i] for i in sorted_indices]

        cat_population = sorted_population[num_mice:]
        mouse_population = sorted_population[:num_mice]
        cat_drone_occupancies = sorted_drone_occupancies[num_mice:]
        mouse_drone_occupancies = sorted_drone_occupancies[:num_mice]

        # Update cat positions
        cat_population = update_cat_positions(cat_population, mouse_population, sorted_objective_values[num_mice:],starting_points, target_points, cat_drone_occupancies, grid)

        # Update mouse positions
        havens, _ , _  = generate_population(grid_size, starting_points, target_points, obstacles)
        mouse_population = update_mouse_positions(mouse_population, havens, sorted_objective_values[:num_mice],starting_points, target_points, mouse_drone_occupancies, grid)

        # Combine updated populations
        population_matrix = np.vstack((mouse_population, cat_population))

    # Return the best solution found
    best_solution_index = np.argmin(fitness_values)
    best_solution = population_matrix[best_solution_index, :]

    return best_solution


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

# Run CMBO
best_position = cmbo_algorithm(size_of_grid1, ps_list1, pt_list1, obstacle_list1)

