import numpy as np
from queue import Queue

minimum_collision_distance = 1  # Minimum distance between any two points to avoid collisions
maximum_energy = 100  # Maximum allowable total energy consumption
constant_speed = 1.0  # Constant speed of the drones
energy_weight = 0.1  # Weight for energy consumption in the fitness function
energy_per_distance = 1.0  # Energy consumed per unit distance traveled
size_of_grid = 50  # Size of the 3D grid representation of the environment
tolerance = 1.0  # douglas_peucker


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


def calculate_single_path_distance(x):
    """
    Calculate the total distance for a single path x.

    Parameters:
        x (list of tuples): A list of control points defining the path.

    Returns:
        float: The total distance traveled along the path from ps to pt.

    """

    num_points = len(x)

    total_distance = 0
    # Distance between all control points
    for i in range(0, num_points - 1):
        total_distance += euclidean_distance(x[i], x[i + 1])

    total_distance += euclidean_distance(x[-2], x[-1])  # Distance from the last control point to the target

    return total_distance


def calculate_single_path_fitness(x):
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

    total_distance = calculate_single_path_distance(x)

    # Calculate energy consumption based on distance
    energy_consumed = total_distance * energy_per_distance

    # Calculate the fitness value as a weighted sum of time and energy
    fitness = total_distance + energy_weight * energy_consumed

    return fitness


def calculate_total_fitness(drone_paths):
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
        fitness = calculate_single_path_fitness(path)
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
    return check_collision_constraint(drone_paths, obstacle_list) and check_energy_constraint(ps_list, pt_list,
                                                                                              drone_paths)


def check_feasibility_SA(drone_paths, obstacle_list, r1, r2):
    """
    Check if the solution is feasible.

    Args:

    drone_paths (list): List of drone paths where each path is a list of 3D points.
    obstacle_list (list): List of obstacle representations.


    Returns:
        bool: True if the solution is feasible, False otherwise.
    """

    for i in range(len(drone_paths)):
        if i == r1:
            continue
        path1 = drone_paths[i]
        point = drone_paths[r1][r2]
        for point1 in path1:
            if euclidean_distance(point1, point) < minimum_collision_distance:
                return False

    for obstacle in obstacle_list:
        point = drone_paths[r1][r2]

        for i in range(obstacle[0][0], obstacle[1][0]):
            for j in range(obstacle[0][1], obstacle[1][1]):
                for k in range(obstacle[0][2], obstacle[1][2]):
                    if euclidean_distance(point, (i, j, k)) < minimum_collision_distance:
                        return False
    return True


def build_grid(obstacles, size_of_grid):
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


def get_single_path_with_bfs(ps, pt, grid, drone_occupancy):
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
        return 0 <= x < len(grid) and 0 <= y < len(grid) and 0 <= z < len(grid) and grid[x][y][z] == 0 and (
                    drone_occupancy[x][y][z] == (0, 0) or drone_occupancy[x][y][z][1] != depth)

    def get_neighbors(point):
        x, y, z = point
        neighbors = [(x + dx, y + dy, z + dz) for dx in [-1, 0, 1] for dy in [-1, 0, 1] for dz in [-1, 0, 1] if
                     (dx != 0 or dy != 0 or dz != 0)]
        return [neighbor for neighbor in neighbors if is_valid(neighbor)]

    start = tuple(map(int, ps))
    target = tuple(map(int, pt))
    queue = Queue()
    queue.put(start)
    came_from = {}
    came_from[start] = None
    depth = 0

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
        depth += 1

    return []


def get_all_paths_with_bfs(ps_list, pt_list, grid):
    """
    Get all paths from the start points to the target points using BFS.

    Args:
        ps (tuple): The start point (x, y, z).
        pt (tuple): The target point (x, y, z).
        grid (list): A 3D grid representation of the environment.

    Returns:
        list: A list of paths, where each path is a list of control points.
    """
    all_paths = []
    simplified_paths = []

    drone_occupancy = [[[(0, 0) for _ in range(len(grid))] for _ in range(len(grid))] for _ in range(len(grid))]

    drone_tag = 0
    for ps, pt in zip(ps_list, pt_list):
        path = get_single_path_with_bfs(ps, pt, grid, drone_occupancy)
        simplified_path = douglas_peucker(path)
        all_paths.append(path)
        simplified_paths.append(simplified_path)

        depth = 0
        for point in path:
            x, y, z = point
            drone_occupancy[x][y][z] = (drone_tag, depth)
            depth += 1

        drone_tag += 1

    return simplified_paths


def generate_initial_solution(ps_list, pt_list, grid):
    """
    Generate an initial solution to the problem using BFS.

    Args:
        ps (tuple): The start point (x, y, z).
        pt (tuple): The target point (x, y, z).

    Returns:
        list: Initial solution.
    """
    return get_all_paths_with_bfs(ps_list, pt_list, grid)


def euclidean_distance(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2) ** 0.5


def douglas_peucker(points):
    """
    Applys douglas_peucker algorithm on generated points from bfs to lower the number of control points

    Args:
        points (list): A list of all paths generated by BFS.

    Returns:
        list: similar path with fewer number of control points.
    """
    print(points)
    if len(points) <= 2:
        return [points[0], points[-1]]

    # Find the point farthest from the line connecting the start and end points
    max_distance = 0
    max_index = 0
    start, end = points[0], points[-1]

    for i in range(1, len(points) - 1):
        distance = euclidean_distance(points[i], start) + euclidean_distance(points[i], end)
        line_distance = euclidean_distance(start, end)
        d = distance - line_distance

        if d > max_distance:
            max_distance = d
            max_index = i

    # Check if the farthest point exceeds the tolerance
    if max_distance > tolerance:
        # Recursively simplify the path
        first_segment = douglas_peucker(points[:max_index + 1])
        second_segment = douglas_peucker(points[max_index:])

        return first_segment[:-1] + second_segment  # Exclude the duplicated point
    else:
        return [start, end]
    