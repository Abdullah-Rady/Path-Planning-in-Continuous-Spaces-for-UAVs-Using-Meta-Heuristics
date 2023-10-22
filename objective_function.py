import numpy as np
from queue import Queue


minimum_collision_distance = 1 # Minimum distance between any two points to avoid collisions
maximum_energy = 100 # Maximum allowable total energy consumption
constant_speed = 1.0 # Constant speed of the drones
energy_weight = 0.1 # Weight for energy consumption in the fitness function
energy_per_distance = 1.0 # Energy consumed per unit distance traveled
size_of_grid = 50 # Size of the 3D grid representation of the environment

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


def calculate_single_path_distance(x, ps, pt):
    """
    Calculate the total distance for a single path x.
    
    Parameters:
        x (list of tuples): A list of control points defining the path.
        ps (tuple): The start point (x, y, z).
        pt (tuple): The target point (x, y, z).

    Returns:
        float: The total distance traveled along the path from ps to pt.

    """
     
    num_points = len(x)

    total_distance = euclidean_distance(ps, x[0])  # Distance from start to the first control point

    # Distance between all control points
    for i in range(1, num_points - 1):
        total_distance += euclidean_distance(x[i], x[i + 1])

    total_distance += euclidean_distance(x[-1], pt)  # Distance from the last control point to the target

    return total_distance

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

    total_distance += calculate_single_path_distance(x, ps, pt)


    # Calculate energy consumption based on distance
    energy_consumed = total_distance * energy_per_distance

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

    Returns:
        bool: True if the energy constraint is satisfied, False otherwise.
    """

    for path, psi, pti in zip(drone_paths, ps, pt):
        total_distance = calculate_single_path_distance(path, psi, pti)
        energy_consumed = total_distance * energy_per_distance

        if energy_consumed > maximum_energy:
            return False  # Energy constraint is violated
        
    return True  # Energy constraint is satisfied

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

def build_grid(obstacles):

    """
    Build a grid representation of the environment.
        0: Free space
        1: Obstacle

    Args:
        ps (tuple): The start point (x, y, z).
        pt (tuple): The target point (x, y, z).
        obstacles (list): List of obstacle representations.

    Returns:
        list: A 3D grid representation of the environment.
        

    """
    grid = [[[0 for _ in range(size_of_grid)] for _ in range(size_of_grid)] for _ in range(size_of_grid)]

    # Set all obstacles to 1
    for obstacle in obstacles:
       for i in range(obstacle[0][0], obstacle[1][0]):
              for j in range(obstacle[0][1], obstacle[1][1]):
                    for k in range(obstacle[0][2], obstacle[1][2]):
                        grid[i][j][k] = 1

    return grid



def get_single_path_with_bfs(ps, pt, grid):
    """
    Get a path from the start point to the target point using BFS.

    Args:
        ps (tuple): The start point (x, y, z).
        pt (tuple): The target point (x, y, z).
        grid (list): A 3D grid representation of the environment.

    Returns:
        list: A list of control points defining the path.
    """
    def is_valid(point):
        x, y, z = point
        return 0 <= x < size_of_grid and 0 <= y < size_of_grid and 0 <= z < size_of_grid and grid[x][y][z] == 0

    def get_neighbors(point):
        x, y, z = point
        neighbors = [(x + dx, y + dy, z + dz) for dx in [-1, 0, 1] for dy in [-1, 0, 1] for dz in [-1, 0, 1] if (dx != 0 or dy != 0 or dz != 0)]
        return [neighbor for neighbor in neighbors if is_valid(neighbor)]

    start = tuple(map(int, ps))
    target = tuple(map(int, pt))
    queue = Queue()
    queue.put(start)
    came_from = {}
    came_from[start] = None

    while not queue.empty():
        current = queue.get()
        if current == target:
            path = [current]
            while current != start:
                current = came_from[current]
                path.insert(0, current)
            return path

        for neighbor in get_neighbors(current):
            if neighbor not in came_from:
                queue.put(neighbor)
                came_from[neighbor] = current

    return []

def get_all_paths_with_bfs(ps_list, pt_list, grid):
    """
    Get all paths from the start points to the target points using BFS.

    Returns:
        list: A list of paths, where each path is a list of control points.
    """
    all_paths = []

    for ps, pt in zip(ps_list, pt_list):
        path = get_single_path_with_bfs(ps, pt, grid)
        all_paths.append(path)

    return all_paths

def generate_initial_solution(ps_list, pt_list, obstacles):
    """
    Generate an initial solution to the problem using BFS.

    Returns:
        list: Initial solution.
    """
    grid = build_grid(obstacles)
    return get_all_paths_with_bfs(ps_list, pt_list, grid)



# Example usage:
# Define paths for three drones
drone1_path = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (2, 1, 0)]
drone2_path = [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 2, 0)]
drone3_path = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (2, 1, 0)]

# Define start and target points for the drones

# Define start points for the drones (x, y, z)
ps_list = [(0, 0, 5), (5, 0, 3), (1, 1, 2)]

# Define target points for the drones (x, y, z)
pt_list = [(5, 6, 4), (0, 8, 6), (5, 4, 1)]

# Define obstacles [(x, y, z) (x, y, z)]
obstacle_list = [[(2, 1, 1), (3, 2, 6)], [(2, 3, 1), (3, 6, 6)]]

# Constant speed of drones (e.g., 1.0 units per time step)
constant_speed = 1.0

# Weight for energy consumption in the fitness function
energy_weight = 0.1

# Create a list of drone paths
drone_paths = [drone1_path, drone2_path, drone3_path]

# Calculate the total fitness for all drones
total_fitness = calculate_total_fitness(drone_paths, ps_list, pt_list, constant_speed, energy_weight)
print("Total Fitness for All Drones:", total_fitness)