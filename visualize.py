import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import json
import time


def visualize_problem_solution(ps_list, pt_list, obstacle_list, solution_paths):
    # Function to plot points
    def plot_points(points, ax, color, marker):
        for point in points:
            ax.scatter(*point, c=color, marker=marker, s=100)

    # Function to plot lines between start and end points
    def plot_lines(start_points, end_points, ax):
        for start, end in zip(start_points, end_points):
            ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]], c='black', linewidth=1)

    # Function to plot obstacles
    def plot_obstacles(obstacles, ax):
        for obstacle in obstacles:
            starting_point = obstacle[0]
            ending_point = obstacle[1]
            obstacle_points = []
            for x_value in range(starting_point[0], ending_point[0] + 1):
                for y_value in range(starting_point[1], ending_point[1] + 1):
                    for z_value in range(starting_point[2], ending_point[2] + 1):
                        obstacle_points.append((x_value, y_value, z_value))
            plot_points(obstacle_points, ax, 'red', 's')

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the grid
    # plot_grid(size_of_grid1, ax)

    colors = ['green', 'blue', 'red']

    if solution_paths is not None:
        for i, path in enumerate(solution_paths):
            for point in path:
                ax.plot(*point, c=colors[i], marker='_', linewidth=1)

    # Plot lines connecting start and end points
    plot_lines(ps_list, pt_list, ax)

    plot_points(ps_list, ax, 'blue', 'x')  # Plot start points
    # Plot target points
    plot_points(pt_list, ax, 'green', 'o')

    # Plot obstacles
    plot_obstacles(obstacle_list, ax)

    # Set labels
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    # Set title
    plt.title('3D Grid with Target Points and Obstacles')

    # Show the plot
    plt.show()
# def visualize_problem_solution(ps_list, pt_list, obstacle_list, solution_paths):

#     # Function to plot points
#     def plot_points(points, ax, color, marker):
#         for point in points:
#             ax.scatter(*point, c=color, marker=marker, s=100)

#     # Function to plot obstacles
#     def plot_obstacles(obstacles, ax):
#         for obstacle in obstacles:
#             starting_point = obstacle[0]
#             ending_point = obstacle[1]
#             obstacle_points =[]
#             for x_value in range(starting_point[0],ending_point[0]+1):
#                 for y_value in range(starting_point[1],ending_point[1]+1):
#                     for z_value in range(starting_point[2],ending_point[2]+1):
#                         obstacle_points.append((x_value,y_value,z_value))
#             plot_points(obstacle_points, ax, 'red', 's')
#     # Create a 3D plot
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')

#     # Plot the grid
#     # plot_grid(size_of_grid1, ax)

#     colors = ['green', 'blue', 'red']

#     if(solution_paths != None):
#         for i, path in enumerate(solution_paths):
#             for point in path:
#                 ax.plot(*point, c=colors[i], marker='_', linewidth=1)

#     plot_points(ps_list, ax, 'blue', 'x')  # Plot start points
#     # Plot target points
#     plot_points(pt_list, ax, 'green', 'o')

#     # Plot obstacles
#     plot_obstacles(obstacle_list, ax)

#     # Set labels
#     ax.set_xlabel('X-axis')
#     ax.set_ylabel('Y-axis')
#     ax.set_zlabel('Z-axis')

#     # Set title
#     plt.title('3D Grid with Target Points and Obstacles')

#     # Show the plot
#     plt.show()

def plot_fitness_over_iterations( fitness_values):
    """
    Plot the best fitness over iterations.

    Parameters:
    - fitness_values: List of corresponding best fitness values.
    """
    window_size = 10
    mean_per_population = calculate_stats_per_window(fitness_values, window_size)

    plt.plot(np.range(1, len(fitness_values) // window_size), mean_per_population, marker='o', linestyle='-')
    plt.xlabel('Generation')
    plt.ylabel('Mean Fitness')
    plt.title('Genetic Algorithm: Mean Fitness over Generations')
    plt.grid(True)
    plt.show()

def plot_best_fitness_over_iterations(fitness_values):
    """
    Plot the best fitness over iterations.

    Parameters:
    - iteration_list: List of iteration numbers.
    - fitness_values: List of corresponding best fitness values.
    """
    window_size = 10
    min_per_population, prefix_min = calculate_min_stats_per_window(fitness_values, window_size)

    generations = np.arange(1, len(fitness_values) // window_size, window_size)

    fig, axs = plt.subplots(2, figsize=(8, 8))

    axs[0].plot(generations, min_per_population, marker='o', linestyle='-')
    axs[0].set_xlabel('Generation')
    axs[0].set_ylabel('Min Fitness')
    axs[0].set_title('Genetic Algorithm: Min Fitness per Generation')
    axs[0].grid(True)

    axs[1].plot(generations, prefix_min, marker='o', linestyle='-')
    axs[1].set_xlabel('Generation')
    axs[1].set_ylabel('Min Fitness')
    axs[1].set_title('Genetic Algorithm: Prefix Min Fitness per Generations')
    axs[1].grid(True)

    plt.tight_layout()
    plt.show()

def calculate_stats_per_window(fitness_values, window_size):

    mean_per_window = []
    std_per_window = []

    num_windows = len(fitness_values) // window_size

    for i in range(num_windows):
        window_start = i * window_size
        window_end = window_start + window_size
        window_fitness = fitness_values[window_start:window_end]
        window_mean = np.mean(window_fitness)
        window_std = np.std(window_fitness)
        mean_per_window.append(window_mean)
        std_per_window.append(window_std)

    return mean_per_window, std_per_window

def calculate_min_stats_per_window(fitness_values, window_size):

    min_per_window = []

    num_windows = len(fitness_values) // window_size
    prefix_min = float('inf')
    prefix_min_array = []

    for i in range(num_windows):
        window_start = i * window_size
        window_end = window_start + window_size
        window_fitness = fitness_values[window_start:window_end]
        min_fitness = np.min(window_fitness)
        prefix_min = min(prefix_min, min_fitness)
        prefix_min_array.append(prefix_min)
        min_per_window.append(min_fitness)

    return min_per_window, prefix_min_array


def calculate_stats(fitness_values, start_time, end_time):
    fitness_mean = np.mean(fitness_values)
    fitness_std = np.std(fitness_values)
    runtime = end_time - start_time
    return {
        "mean_fitness": fitness_mean,
        "std_fitness": fitness_std,
        "runtime": runtime
    }

def save_scenario_stats_to_json(scenario_name, stats_dict):
    with open(f"{scenario_name}_stats.json", "w") as file:
        json.dump(stats_dict, file)

def compile_scenario_stats_into_table(scenario_names):
    data = []
    for scenario_name in scenario_names:
        with open(f"{scenario_name}_stats.json", "r") as file:
            stats_dict = json.load(file)
            stats_dict["scenario"] = scenario_name
            data.append(stats_dict)

