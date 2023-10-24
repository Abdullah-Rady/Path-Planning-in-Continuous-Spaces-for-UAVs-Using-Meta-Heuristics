import random
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from objective_function import calculate_total_fitness, generate_initial_solution, check_feasibility, \
    check_feasibility_SA, build_grid

max_iterations = 5000  # Maximum number of iterations
n_iterations = 100  # Number of iterations at each temperature

initial_temperature = 1000.0  # Initial temperature
final_temperature = 0.1  # Final temperature

alpha = 0.9  # Cooling rate
beta = (final_temperature - initial_temperature) / max_iterations  # Cooling rate


def geometric_cooling_schedule(T_current):
    return T_current * alpha


def linear_cooling_schedule(T_current):
    return T_current - beta


def simulated_annealing(ps_list, pt_list, grid):
    current_solution = generate_initial_solution(ps_list, pt_list, grid)  # Initial solution is the simplified path

    print(current_solution)

    best_solution = current_solution  # Best solution found so far

    best_solution_value = calculate_total_fitness(current_solution)  # Objective value of the best solution

    current_temperature = initial_temperature
    iteration = 0

    while current_temperature > final_temperature and not iteration > max_iterations:
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
            x_new = list(path[r2])  # Make a copy of the selected point

            # Modify the x, y, z values as needed, e.g., add random perturbation
            x_new[0] += random.uniform(-1, 1)
            x_new[1] += random.uniform(-1, 1)
            x_new[2] += random.uniform(-1, 1)

            # Create a new solution by replacing the selected point in the path
            new_solution = current_solution[:]
            new_solution[r1] = path[:r2] + [tuple(x_new)] + path[r2 + 1:]

            if not check_feasibility_SA(new_solution, obstacle_list, r1, r2):
                continue

            # Calculate the change in energy (objective value)
            delta_E = calculate_total_fitness(new_solution) - fitness_value

            if delta_E < 0:
                # Accept the better solution
                current_solution = new_solution
            elif random.random() < math.exp(-delta_E / current_temperature):
                # Accept the solution with a probability
                current_solution = new_solution

            # Update the best solution if necessary
            if calculate_total_fitness(current_solution) < best_solution_value:
                best_solution = current_solution
                best_solution_value = calculate_total_fitness(current_solution)

        # Update temperature and iteration counter
        current_temperature = geometric_cooling_schedule(current_temperature)
        iteration += 1
    # Return the best solution found
    return best_solution


def plot_graph(fitness_values_per_iteration, min_fitness_values):
    def update(frame):
        plt.clf()  # Clear the current frame

        # Create two subplots side by side
        plt_iteration = frame + 1

        # Plot for Total Fitness
        ax1 = plt.subplot(2, 2, 1)
        ax1.plot(range(1, plt_iteration + 1), fitness_values_per_iteration[:plt_iteration], marker='o',
                 linestyle='-')
        ax1.set_ylabel('Total Fitness')
        ax1.set_title('Total Fitness Evolution Over Generations')

        # Plot for Minimum Fitness
        ax2 = plt.subplot(2, 2, 2)
        ax2.plot(range(1, plt_iteration + 1), min_fitness_values[:plt_iteration], marker='o', linestyle='-')
        ax2.set_xlabel('Generation')
        ax2.set_ylabel('Minimum Fitness')
        ax2.set_title('Minimum Fitness Evolution Over Generations')

        # Create the initial plot with two subplots

    fig = plt.figure(figsize=(12, 5))

    plt.subplots_adjust(hspace=0.5)

    fig.canvas.manager.window.wm_geometry("+50+100")

    # Create the animation
    ani = FuncAnimation(fig, update, frames=len(fitness_values_per_iteration), repeat=False, blit=False)

    plt.show()


# Example usage:

numberOfDrones = 3  # Number of drones
numberOfDrones1 = 4

size_of_grid = 50  # Size of the grid
size_of_grid1 = 30  # Size of the grid

# Define start points for the drones (x, y, z)
ps_list = [(0, 0, 5), (5, 0, 3), (1, 1, 2)]
ps_list1 = [(5, 5, 5), (10, 10, 10), (20, 20, 20), (5, 20, 10)]

# Define target points for the drones (x, y, z)
pt_list = [(5, 6, 4), (0, 8, 6), (5, 4, 1)]
pt_list1 = [(25, 25, 25), (5, 15, 20), (18, 12, 12), (10, 25, 15)]

# Define obstacles [(x, y, z) (x, y, z)] all grid cells from x1 to x2 and y1 to y2 and z1 to z2 are obstacles
obstacle_list = [[(2, 1, 1), (3, 2, 6)], [(2, 3, 1), (3, 6, 6)]]
obstacle_list1 = [[(8, 8, 8), (12, 12, 12)], [(20, 15, 10), (25, 18, 20)], [(7, 15, 12), (10, 20, 18)]]

grid = build_grid(obstacle_list1, size_of_grid1)  # Build grid
best_solution = simulated_annealing(ps_list1, pt_list1, grid)  # Run simulated annealing

print(best_solution)  # Print best solution

