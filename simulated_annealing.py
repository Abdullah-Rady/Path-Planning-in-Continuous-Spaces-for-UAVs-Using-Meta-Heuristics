import random
import math

from objective_function import calculate_total_fitness,check_feasibility


numberOfDrones = 3  # Number of drones
numberOfControlPoints = 5  # Number of control points
initialTemperature = 100.0  # Initial temperature
finalTemperature = 0.1  # Final temperature
max_iterations = 1000  # Maximum number of iterations
n_iterations = 100  # Number of iterations at each temperature

size_of_field = 50  # Size of the field

starting_points = [(),(),()] # List of starting points
target_points = [(),(),()] # List of ending points
obstacles = [(),(),()] # List of obstacles




def generateInitialSolution():
    """
    Generate an initial solution to the problem.

    Returns:
        list: Initial solution.
    """
    # Generate a random list of the following form [(x1,y1,z1),(x2,y2,z2),...]
    # where (xi,yi,zi) is the position of the ith drone
    while True:
        initial_solution = []
        for _ in range(numberOfDrones):
            drone_points = []
            for _ in range(numberOfControlPoints):
                x = random.uniform(0, size_of_field)
                y = random.uniform(0, size_of_field)
                z = random.uniform(0, size_of_field)
                drone_points.append((x, y, z))
            initial_solution.append(drone_points)
        if(check_feasibility(starting_points,target_points,initial_solution,obstacles)):
            break
    return initial_solution




def simulated_annealing():
    current_solution = generateInitialSolution  # Initial solution
    best_solution = current_solution  # Best solution found so far
    best_solution_value = calculate_total_fitness(current_solution,starting_points,target_points)  # Objective value of the best solution
    currentTemperature = initialTemperature 
    iteration = 0

    while currentTemperature > finalTemperature and iteration < max_iterations:
        for _ in range(n_iterations):
            fitness_value = calculate_total_fitness(x,starting_points,target_points)
            # Generate a new solution x_new
            x_new = [x_current[i] + random.uniform(-1, 1) for i in range(len(x_current))]

            # Calculate the change in energy (objective value)
            delta_E = calculate_total_fitness(x_new,starting_points,target_points) - calculate_total_fitness(x_new,starting_points,target_points)

            if delta_E < 0:
                # Accept the better solution
                x_current = x_new
            elif random.random() < math.exp(-delta_E / T_current):
                # Accept the solution with a probability
                x_current = x_new

            # Update the best solution if necessary
            if calculate_total_fitness(x_current,starting_points,target_points) < f_best:
                x_best = x_current
                f_best = objective_function(x_current)

        # Update temperature and iteration counter
        T_current = cooling_schedule(T_current)
        iteration += 1

# Output the best solution found
print("Best Solution:", x_best)
print("Best Objective Value:", f_best)
