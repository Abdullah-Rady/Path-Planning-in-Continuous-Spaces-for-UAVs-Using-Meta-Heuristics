import numpy as np


minimum_collision_distance = 1 # Minimum distance between any two points to avoid collisions
maximum_energy = 100 # Maximum allowable total energy consumption
constant_speed = 1.0 # Constant speed of the drones
energy_weight = 0.1 # Weight for energy consumption in the fitness function


def euclidean_distance(point1, point2):
    """
    Calculate the Euclidean distance between two 3D points.

    Parameters:
    point1 (tuple): The first 3D point (x, y, z).
    point2 (tuple): The second 3D point (x, y, z).

    Returns:
    float: The Euclidean distance between point1 and point2.
    """
    return np.linalg.norm(np.array(point1) - np.array(point2))

def calculate_single_path_fitness(x, ps, pt):
    """
    Calculate the total distance and energy consumption for a single path x.

    Parameters:
    x (list of tuples): A list of control points defining the path.
    ps (tuple): The start point (x, y, z).
    pt (tuple): The target point (x, y, z).

    Returns:
        float: The fitness value for the path.
    """
    num_points = len(x)
    if num_points < 2:
        return 0.0  # No movement if there are no control points

    total_distance = euclidean_distance(ps, x[0])  # Distance from start to the first control point

    # Distance between all control points
    for i in range(1, num_points - 1):
        total_distance += euclidean_distance(x[i], x[i + 1])

    total_distance += euclidean_distance(x[-1], pt)  # Distance from the last control point to the target


    # Calculate energy consumption based on distance and constant speed
    energy_consumed = total_distance / constant_speed

    # Calculate the fitness value as a weighted sum of time and energy
    fitness = total_distance + energy_weight * energy_consumed

    return fitness


def calculate_total_fitness(drone_paths, ps_list, pt_list):
    """
    Calculate the total distance for multiple drones' paths.

    Parameters:
    drone_paths (list of lists): A list of paths, each defined as a list of control points.
    ps_list (list of tuples): A list of start points for each drone.
    pt_list (list of tuples): A list of target points for each drone.

    Returns:
    float: The total fitness value traveled along all paths from ps to pt.
    """
    total_fitness = 0.0
    for i in range(len(drone_paths)):
        path = drone_paths[i]
        ps = ps_list[i]
        pt = pt_list[i]
        fitness = calculate_single_path_fitness(path, ps, pt)
        total_fitness += fitness

    return total_fitness

def check_energy_constraint(ps, pt, drone_paths):
    """
    Check if the total energy consumed by all drones satisfies the energy constraint.

    Args:
        drone_paths (list): List of drone paths where each path is a list of 3D points.
        constant_speed (float): Constant speed of the drones.
        maximum_energy (float): Maximum allowable total energy consumption.

    Returns:
        bool: True if the energy constraint is satisfied, False otherwise.
    """
    total_energy = 0.0
    for path, psi, pti in zip(drone_paths, ps, pt):
        total_distance = calculate_single_path_fitness(path, psi, pti)
        energy_consumed = total_distance / constant_speed
        total_energy += energy_consumed

    if total_energy <= maximum_energy:
        return True  # Energy constraint is satisfied
    else:
        return False  # Energy constraint is violated

def check_collision_constraint(drone_paths, obstacle_list):
    """
    Check if drone paths collide with each other or with obstacles.

    Args:
        drone_paths (list): List of drone paths where each path is a list of 3D points.
        obstacle_list (list): List of obstacle representations.

    Returns:
        bool: True if the collision constraint is satisfied, False otherwise.
    """
    for i in range(len(drone_paths)):
        for j in range(i + 1, len(drone_paths)):
            path1 = drone_paths[i]
            path2 = drone_paths[j]
            for point1 in path1:
                for point2 in path2:
                    if euclidean_distance(point1, point2) < minimum_collision_distance:
                        return False  # Collisions detected

    for path in drone_paths:
        for point in path:
            for obstacle in obstacle_list:
                if euclidean_distance(point, obstacle) < minimum_collision_distance:
                    return False  # Collisions with obstacles detected

    return True  # No collisions with other drones or obstacles

def check_feasibility(ps_list, pt_list, drone_paths, obstacle_list):
    """
    Check if the solution is feasible.

    Args:
    ps_list (list of tuples): A list of start points for each drone.
    pt_list (list of tuples): A list of target points for each drone.
    drone_paths (list): List of drone paths where each path is a list of 3D points.
    obstacle_list (list): List of obstacle representations.


    Returns:
        bool: True if the solution is feasible, False otherwise.
    """
    return check_collision_constraint(drone_paths, obstacle_list) and check_energy_constraint(ps_list, pt_list, drone_paths)



# # Example usage:
# # Define paths for three drones
# drone1_path = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (2, 1, 0)]
# drone2_path = [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 2, 0)]
# drone3_path = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (2, 1, 0)]

# # Define start and target points for the drones
# ps_list = [(0, 0, 0), (0, 0, 0), (0, 0, 0)]
# pt_list = [(2, 2, 0), (2, 2, 0), (2, 2, 0)]

# # Constant speed of drones (e.g., 1.0 units per time step)
# constant_speed = 1.0

# # Weight for energy consumption in the fitness function
# energy_weight = 0.1

# # Create a list of drone paths
# drone_paths = [drone1_path, drone2_path, drone3_path]

# # Calculate the total fitness for all drones
# total_fitness = calculate_total_fitness(drone_paths, ps_list, pt_list, constant_speed, energy_weight)
# print("Total Fitness for All Drones:", total_fitness)