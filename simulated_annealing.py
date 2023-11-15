import random
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D


from objective_function import calculate_total_fitness, generate_initial_solution,check_feasibility_SA, build_grid

# Simulated Annealing Parameters

max_iterations = 500  # Maximum number of iterations
n_iterations = 10  # Number of iterations at each temperature
initial_temperature = 100.0  # Initial temperature
final_temperature = 0.1  # Final temperature

alpha = 0.9  # Geometric Cooling rate
beta = (final_temperature - initial_temperature) / max_iterations  # Linear Cooling rate

#Global Variables
grid = None
drone_occupancy = None


def geometric_cooling_schedule(T_current):
    return T_current * alpha


def linear_cooling_schedule(T_current):
    return T_current - beta


def simulated_annealing(size_of_grid, starting_points, target_points, obstacles):
    best_solution_values = []
    all_solutions_values = []
    temperature_values = []


    current_solution,grid,drone_occupancy = generate_initial_solution(size_of_grid, starting_points, target_points, obstacles)  # Initial solution is the simplified path

    best_solution = current_solution  # Best solution found so far
    best_solution_value = calculate_total_fitness(current_solution)  # Objective value of the best solution

    current_temperature = initial_temperature
    iteration = 0

    while current_temperature > final_temperature and iteration <= max_iterations:
        for _ in range(n_iterations):
            fitness_value = calculate_total_fitness(current_solution)
            # Generate a new solution x_new
            # r1 is a random index of a path, and r2 is a random index of a point within the path
            r1 = random.randint(0, len(current_solution) - 1)
            path = current_solution[r1]

            if len(path) <= 2:
                continue

            r2 = random.randint(1, len(path) - 2)

            # Generate new x, y, z coordinates for the selected point
            new_point = list(path[r2])  # Make a copy of the selected point

            # Modify the x, y, z values as needed, e.g., add random perturbation
            new_point[0] += random.uniform(-1, 1)
            new_point[1] += random.uniform(-1, 1)
            new_point[2] += random.uniform(-1, 1)

            # Create a new solution by replacing the selected point in the path
            new_solution = current_solution[:]
            new_solution[r1] = path[:r2] + [tuple(new_point)] + path[r2 + 1:]

            if not check_feasibility_SA(r1, grid,drone_occupancy, path[r2], new_solution[r1][r2]):
                continue

            # Calculate the change in energy (objective value)
            new_fitness_value = calculate_total_fitness(new_solution)
            delta_E = new_fitness_value - fitness_value
            if delta_E < 0 or random.random() < math.exp(-delta_E / current_temperature):
                current_solution = new_solution
                if new_fitness_value < best_solution_value:
                    best_solution = current_solution
                    best_solution_value = new_fitness_value

        # Update temperature and iteration counter
        current_temperature = geometric_cooling_schedule(current_temperature)
        iteration += 1
        best_solution_values.append(best_solution_value)
        temperature_values.append(current_temperature)

    # Return the best solution found
    return all_solutions_values, best_solution_values, best_solution,temperature_values


def plot_graph(fitness_values_per_iteration, min_fitness_values, drone_paths, temperatures):
    def update(frame):
        plt.clf()  # Clear the current frame

        plt_iteration = frame + 1

        # Plot for Total Fitness
        ax1 = plt.subplot(2, 2, 1)
        ax1.plot(range(1, plt_iteration + 1), fitness_values_per_iteration[:plt_iteration], marker='o', linestyle='-')
        ax1.set_xlabel('Iteration')
        ax1.set_ylabel('Total Cost')
        ax1.set_title('Total Cost Change Over Iterations')

        # Plot for Minimum Fitness
        ax2 = plt.subplot(2, 2, 2)
        ax2.plot(range(1, plt_iteration + 1), min_fitness_values[:plt_iteration], marker='o', linestyle='-')
        ax2.set_xlabel('Iteration')
        ax2.set_ylabel('Minimum Cost')
        ax2.set_title('Minimum Cost Change Over Iterations')

        # Plot Temperature vs. Time
        ax3 = plt.subplot(2, 2, 3)
        ax3.plot(range(1, plt_iteration + 1), temperatures[:plt_iteration], marker='o', linestyle='-', color='r')
        ax3.set_xlabel('Iterations')
        ax3.set_ylabel('Temperature')
        ax3.set_title('Temperature Over Time')

        # Plot 3D Paths for Drones
        # ax4 = plt.subplot(2, 2, 4, projection='3d')
        # for i, path in enumerate(drone_paths[:plt_iteration]):
        #     path = list(zip(*path))  # Transpose the path data
        #     x, y, z = path
        #     ax4.plot(x, y, z, label=f'Drone {i + 1}')

        # ax4.set_xlabel('X')
        # ax4.set_ylabel('Y')
        # ax4.set_zlabel('Z')
        # ax4.set_title('3D Paths for Drones')
        # ax4.legend()

    fig = plt.figure(figsize=(12, 10))
    plt.subplots_adjust(hspace=0.5)

    # Set up the animation
    ani = FuncAnimation(fig, update, frames=len(fitness_values_per_iteration), repeat=False, blit=False)

    plt.show()




# Example usage:

size_of_grid1 = 30  # Size of the grid
size_of_grid2 = 20  # Size of the grid
size_of_grid3 = 30  # Size of the grid


# Define start points for the drones (x, y, z)
ps_list = [(0, 0, 5), (5, 0, 3), (1, 1, 2)]
ps_list1 = [(5, 5, 5), (1, 10, 10), (20, 20, 20)]
ps_list2 = [(1, 1, 1), (5, 5, 5)]
ps_list3 = [(5, 5, 5), (20, 20, 20)]


# Define target points for the drones (x, y, z)
pt_list = [(5, 6, 4), (0, 8, 6)]
pt_list1 = [(25, 25, 25), (1, 15, 20), (18, 12, 12)]
pt_list2 = [(15, 15, 15), (18, 18, 18)]
pt_list3 = [(25, 25, 25), (1, 1, 1)]

# Define obstacles [(x, y, z) (x, y, z)] all grid cells from x1 to x2 and y1 to y2 and z1 to z2 are obstacles
obstacle_list = [[(2, 1, 1), (3, 2, 6)], [(2, 3, 1), (3, 6, 6)]]
obstacle_list1 = [[(8, 8, 8), (12, 12, 12)], [(20, 15, 10), (25, 18, 20)], [(7, 15, 12), (10, 20, 18)]]


# afv, mfv, best_solution,temperatures = simulated_annealing(size_of_grid1, ps_list1, pt_list1, obstacle_list1)  # Run simulated annealing



def visualize_problem(ps_list, pt_list, obstacle_list, size_of_grid, solution_paths):
    # Function to plot the grid
    def plot_grid(size_of_grid, ax):
        for i in range(0, size_of_grid + 1):
            ax.plot([i, i], [0, size_of_grid], [0, 0], color='black', linestyle='-', linewidth=1)
            ax.plot([0, size_of_grid], [i, i], [0, 0], color='black', linestyle='-', linewidth=1)
            ax.plot([i, i], [0, size_of_grid], [size_of_grid, size_of_grid], color='black', linestyle='-', linewidth=1)
            ax.plot([0, size_of_grid], [i, i], [size_of_grid, size_of_grid], color='black', linestyle='-', linewidth=1)

    # Function to plot points

    def plot_points(points, ax, color, marker):
        for point in points:
            ax.scatter(*point, c=color, marker=marker, s=100)

    # Function to plot obstacles
    def plot_obstacles(obstacles, ax):
        for obstacle in obstacles:
            starting_point = obstacle[0]
            ending_point = obstacle[1]
            obstacle_points =[]
            for x_value in range(starting_point[0],ending_point[0]+1):
                for y_value in range(starting_point[1],ending_point[1]+1):
                    for z_value in range(starting_point[2],ending_point[2]+1):
                        obstacle_points.append((x_value,y_value,z_value))
            plot_points(obstacle_points, ax, 'red', 's')
    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the grid
    # plot_grid(size_of_grid1, ax)

    colors = ['green', 'blue', 'red']

    print(solution_paths)
    if(solution_paths != None):
        for i, path in enumerate(solution_paths):
            for point in path:
                ax.plot(*point, c=colors[i], marker='_', linewidth=1)

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

c, grid, drone_occupancy = generate_initial_solution(size_of_grid1, ps_list1, pt_list1, obstacle_list1)  # Initial solution is the simplified path
visualize_problem(ps_list1, pt_list1, obstacle_list1, size_of_grid1, c)  # Visualize the problem


# import time
# import numpy as np

# start_time = time.time()

# grid = build_grid(obstacle_list2, size_of_grid2)  # Build grid
# afv, mfv, best_solution, temperature = simulated_annealing(ps_list2, pt_list2, grid)  # Run simulated annealing

# end_time = time.time()
# runtime = end_time - start_time

# # Calculate the mean and standard deviation
# mean = np.mean(afv)
# std_deviation = np.std(afv)

# print(f"Mean: {mean}")
# print(f"Standard Deviation: {std_deviation}")
# print(f"Runtime: {runtime} seconds")

# mean = np.mean(mfv)
# std_deviation = np.std(mfv)

# print(f"Mean: {mean}")
# print(f"Standard Deviation: {std_deviation}")
# print(f"Runtime: {runtime} seconds")

# plot_graph(afv, mfv, best_solution,temperatures=temperatures)


