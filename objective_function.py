import itertools
import random
import numpy as np
import math
import json
from queue import Queue


# Constants
minimum_collision_distance = 1  # Minimum distance between any two points to avoid collisions
maximum_energy = 100  # Maximum allowable total energy consumption per drone
constant_speed = 1.0  # Constant speed of the drones
energy_weight = 1  # Weight for energy consumption in the fitness function
energy_per_distance = 1.0  # Energy consumed per unit distance traveled
tolerance = 1.0  # douglas_peucker
separation = 1  # separation between drones
min_number_control_points = 1
max_number_control_points = 3




def bresenham3D(point1, point2):
    points = []
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    # Calculate differences and absolute differences
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    gcd_abc = math.gcd(math.gcd(dx, dy), dz)
    if(gcd_abc == 0):
        return [point1,point2]
    # Divide each integer by the GCD
    simplified_dx = dx // gcd_abc
    simplified_dy = dy // gcd_abc
    simplified_dz = dz // gcd_abc
    # Set the starting point
    x, y, z = x1, y1, z1
    points.append((x1, y1, z1))
    while x != x2 or y != y2 or z != z2:
        # Add the current point to the list of points
        x += simplified_dx
        y += simplified_dy
        z += simplified_dz
        points.append((x, y, z))
    return points

def get_points_in_between(point1, point2):
    points = []
    num_points = max(abs(point1[0] - point2[0]), abs(point1[1] - point2[1]), abs(point1[2] - point2[2]))
    for t in range(0,num_points+1):
        t_normalized = t / num_points
        x = int(point1[0] + t_normalized * (point2[0] - point1[0]))
        y = int(point1[1] + t_normalized * (point2[1] - point1[1]))
        z = int(point1[2] + t_normalized * (point2[2] - point1[2]))
        points.append((x, y, z))
    return points


def euclidean_distance(start_point, end_point):
    return ((start_point[0] - end_point[0]) ** 2 + (start_point[1] - end_point[1]) ** 2 + (start_point[2] - end_point[2]) ** 2) ** 0.5


def calculate_single_path_distance(control_points):
    """
    Calculate the total distance for a single path x.

    Parameters:
        control_points (list of 3D tuples): A list of 3D control points defining the path.

    Returns:
        float: The total distance traveled along the path.

    """
    return sum(euclidean_distance(control_points[i], control_points[i + 1]) for i in range(len(control_points) - 1))


def calculate_single_path_fitness(control_points):
    """
    Calculate the total distance and energy consumption for a single path x.

    Parameters:
    control_points (list of 3D tuples): A list of 3D control points defining the path.

    Returns:
        float: The fitness value for the path.
    """

    num_points = len(control_points)
    if num_points < 2:
        return 0.0  # No movement if there are no control points

    total_distance = calculate_single_path_distance(control_points)

    # Calculate energy consumption based on distance
    energy_consumed = total_distance * energy_per_distance

    # Calculate the fitness value as a weighted sum of time and energy
    fitness = total_distance + energy_weight * energy_consumed

    return fitness


def calculate_total_fitness(drone_paths):
    """
    Calculate the total distance for multiple drones' paths.

    Parameters:
    drone_paths (list of lists  of 3D points): A list of paths, where each path is defined as a list of 3D control points.

    Returns:
    float: The total fitness of the paths.
    """
    total_fitness = 0.0
    for path in drone_paths:
        fitness = calculate_single_path_fitness(path)
        total_fitness += fitness
    return total_fitness


def check_energy_constraint(drone_paths):
    """
    Check if the total energy consumed by all drones satisfies the energy constraint.

    Args:
        drone_paths (list): List of drone paths where each path is a list of 3D points.
        constant_speed (float): Constant speed of the drones.

    Returns:
        bool: True if the energy constraint is satisfied, False otherwise.
    """

    for path in drone_paths:
        total_distance = calculate_single_path_distance(path)
        energy_consumed = total_distance * energy_per_distance

        if energy_consumed > maximum_energy:
            return False  # Energy constraint is violated

    return True  # Energy constraint is satisfied




# def check_feasibility_SA(drone_tag, grid, drone_occupancy, drone_path, old_point_index, new_point,starting_point,target_point):
#     """
#     Check if the solution is feasible.

#     Args:

#     drone_paths (list): List of drone paths where each path is a list of 3D points.
#     obstacle_list (list): List of obstacle representations.
#     path_index (int): Index of the path modified.
#     point_index (int): Index of the point modified in the path.

#     Returns:
#         bool: True if the solution is feasible, False otherwise.
#     """

#     #remove all points where drone_tag exists regardless of layer

#     x, y, z = new_point
#     admissible = 0 <= x < len(grid) and 0 <= y < len(grid) and 0 <= z < len(grid) and grid[x][y][z] == 0

#     if(not admissible):
#         return False
    
#     drone_occupancy_copy = drone_occupancy.copy()
#     point_before_edited_point = drone_path[old_point_index-1]
#     point_after_edited_point = drone_path[old_point_index+1]

#     points = get_points_in_between(point_before_edited_point, point_after_edited_point)
#     for (x, y, z) in points:
#         drones_in_cell = drone_occupancy_copy[x][y][z].copy()
#         for i in range(len(drones_in_cell)):
#             if drones_in_cell[i][0] == drone_tag:
#                 drone_occupancy_copy[x][y][z].remove(drones_in_cell[i])


#     path_before_new_point = get_single_path_with_bfs(point_before_edited_point,new_point,grid,drone_occupancy_copy)
#     if(len(path_before_new_point)<2):
#         return False
    
#     simplified_path_before_new_point = douglas_peucker(path_before_new_point)
#     depth = 2
#     for point in path_before_new_point:
#         if point in [starting_point, target_point]:
#             continue
#         x, y, z = point
#         drone_occupancy_copy[x][y][z].append((drone_tag, depth))
#         depth += 1
#     path_after_new_point = get_single_path_with_bfs(new_point, point_after_edited_point,grid,drone_occupancy_copy)

#     if(len(path_after_new_point)<2):
#         return False
#     simplified_path_after_new_point = douglas_peucker(path_after_new_point)

#     for point in path_after_new_point:
#         if point in [starting_point, target_point]:
#             continue
#         x, y, z = point
#         drone_occupancy_copy[x][y][z].append((drone_tag, depth))
#         depth += 1
#     if (old_point_index + 2 < len(drone_path)):
#         for i in range(old_point_index + 2, len(drone_path), 1):
#             x, y, z = drone_path[i]
#             drones_in_cell = drone_occupancy_copy[x][y][z].copy()
#             for j in range(len(drones_in_cell)):
#                 if drones_in_cell[j][0] == drone_tag:
#                     drone_occupancy_copy[x][y][z].remove(drones_in_cell[j])
#                     drone_occupancy_copy[x][y][z].append((drone_tag, depth))
#                     depth += 1

#     last_index_before = 0
#     drone_occupancy = drone_occupancy_copy
#     for i in range(1, len(simplified_path_before_new_point)):
#         drone_path.insert(i+old_point_index, simplified_path_before_new_point[i])
#         last_index_before = i

#     for i in range(1, len(simplified_path_after_new_point) - 1):
#         drone_path.insert(i + last_index_before + old_point_index, simplified_path_after_new_point[i])
#     return True

# def check_feasibility(drone_tag, grid, drone_occupancy, old_path, new_path, starting_point, target_point):
#     """
#     Check if the solution is feasible.

#     Args:

#     drone_paths (list): List of drone paths where each path is a list of 3D points.
#     obstacle_list (list): List of obstacle representations.
#     path_index (int): Index of the path modified.
#     point_index (int): Index of the point modified in the path.

#     Returns:
#         bool: True if the solution is feasible, False otherwise.
#     """

#     #remove all points where drone_tag exists regardless of layer
#     for new_point in new_path:
#         x, y, z = new_point
#         admissible = 0 <= x < len(grid) and 0 <= y < len(grid) and 0 <= z < len(grid) and grid[x][y][z] == 0
#         if(not admissible):
#             return False
    
#     drone_occupancy_copy = drone_occupancy.copy()
#     # point_before_edited_point = drone_path[old_point_index-1]
#     # point_after_edited_point = drone_path[old_point_index+1]

#     for i in range(len(old_path) - 1, 2):
#         points = bresenham3D(old_path[i], old_path[i+1])
#         for (x, y, z) in points:
#             drones_in_cell = drone_occupancy_copy[x][y][z].copy()
#             for i in range(len(drones_in_cell)):
#                 if drones_in_cell[i][0] == drone_tag and (x, y ,z) != starting_point and (x, y, z) != target_point:
#                     drone_occupancy_copy[x][y][z].remove(drones_in_cell[i])

#     new_control_points = []
#     depth = 0
#     for i in range(len(new_path) - 1, 2):
#         new_path = get_single_path_with_bfs(new_path[i],new_path[i+1],grid,drone_occupancy_copy)
#         if(len(new_path)<2):
#             return False
        
        

#         for p in new_path:
#             x, y, z = p
#             drone_occupancy_copy[x][y][z].append((drone_tag, depth))
#             depth += 1

#         simplified_new_path = douglas_peucker(new_path)
#         for point in simplified_new_path:
#             if (point not in new_control_points and point != starting_point and point != target_point):
#                 new_control_points.append(point)

#     drone_occupancy = drone_occupancy_copy
    
#     return True


def get_closest_valid_point(invalid_point,valid_points):
    min_distance = euclidean_distance(invalid_point,valid_points[0])
    closest_point = valid_points[0]
    for point in valid_points:
        distance = euclidean_distance(invalid_point,point)
        if(distance < min_distance):
            min_distance = distance
            closest_point = point
    return closest_point


def remove_from_occurence(path,drone_occupancy,drone_tag):
        # for i in range(len(path)-1):
        #     first_point = path[i]
        #     second_point = path[i+1]
        #     points_in_between = get_points_in_between(first_point, second_point)
        #     for point in points_in_between:
        #         x, y, z = point
        #         drones_in_cell = drone_occupancy[x][y][z].copy()
        #         for j in range(len(drones_in_cell)):
        #             if drones_in_cell[j][0] == drone_tag:
                        # drone_occupancy[x][y][z].remove(drones_in_cell[j])
        for i in range(len(drone_occupancy)):
            for j in range(len(drone_occupancy)):
                for k in range(len(drone_occupancy)):
                    drones_in_cell = drone_occupancy[i][j][k].copy()
                    for l in range(len(drones_in_cell)):
                        if(drones_in_cell[l][0] == drone_tag):
                            drone_occupancy[i][j][k].remove(drones_in_cell[l])
        return drone_occupancy


def tweak_path_crossover(drone_paths1,drone_paths2,index_path_to_be_tweaked, drone_occupancy1,drone_occupancy2,grid, visualize = False):
    if visualize:
        print("Crossing over after drone: ", index_path_to_be_tweaked + 1)
    new_paths_1 = []
    new_paths_2 = []
    drone_occupancy_copy1 = drone_occupancy1.copy()
    drone_occupancy_copy2 = drone_occupancy2.copy()
    for index_path_to_be_kept in range(index_path_to_be_tweaked):
        new_paths_1.append(drone_paths1[index_path_to_be_kept])
        new_paths_2.append(drone_paths2[index_path_to_be_kept])
    old_path1 = drone_paths1[index_path_to_be_tweaked]
    old_path2 = drone_paths2[index_path_to_be_tweaked]
    drone_occupancy_copy1 = remove_from_occurence(old_path1,drone_occupancy_copy1,index_path_to_be_tweaked+1)
    drone_occupancy_copy2 = remove_from_occurence(old_path2,drone_occupancy_copy2,index_path_to_be_tweaked+1)
    new_path_1 , drone_occupancy_copy1 = tweak_path_cross(drone_paths1,index_path_to_be_tweaked, drone_paths2[index_path_to_be_tweaked],drone_occupancy_copy1,drone_paths2[index_path_to_be_tweaked][0],drone_paths2[index_path_to_be_tweaked][-1],grid, visualize=visualize)
    new_path_2 , drone_occupancy_copy2 = tweak_path_cross(drone_paths2,index_path_to_be_tweaked, drone_paths1[index_path_to_be_tweaked],drone_occupancy_copy2,drone_paths2[index_path_to_be_tweaked][0],drone_paths2[index_path_to_be_tweaked][-1],grid, visualize=visualize)
    if(len(new_path_1) == 0 or len(new_path_2) == 0):
        if visualize:
            print("Crossover failed")
        return [],[],[],[]
    new_paths_1.append(new_path_1)
    new_paths_2.append(new_path_2)
    for index_path_to_be_kept in range(index_path_to_be_tweaked+1,len(drone_paths1)):
        new_paths_1.append(drone_paths2[index_path_to_be_kept])
        new_paths_2.append(drone_paths1[index_path_to_be_kept])
    return new_paths_1,new_paths_2,drone_occupancy_copy1,drone_occupancy_copy2




def tweak_path_cross(drone_paths,index_path_to_be_tweaked, path_to_be_inserted, drone_occupancy,starting_point, target_point, grid, visualize = False):
    if visualize:
        print("Tweaking path for Drone " + str(index_path_to_be_tweaked+1))

    old_path = drone_paths[index_path_to_be_tweaked]
    drone_occupancy_copy = drone_occupancy.copy()
    new_path = []
    new_path.append(starting_point)
   
    valid_points = get_valid_points(grid,starting_point,target_point)
    depth = 0
    path_length = min(len(old_path), len(path_to_be_inserted))
    end = path_length-2
    for i in range(end):
        valid_next_points,point_depths = get_all_valid_next_points(grid,new_path[i],valid_points,drone_occupancy_copy,depth)
        if(i == end - 1):
            previous_valid_points = get_all_valid_previous_points(grid,target_point,valid_next_points,drone_occupancy_copy,point_depths)
            valid_next_points = previous_valid_points
        if(old_path[i+1] in valid_next_points):
            valid_next_points.remove(old_path[i+1])
        if(len(valid_next_points) == 0):
            if visualize:
                print("Path cannot be tweaked")
            return [],[]
        new_point = get_closest_valid_point(path_to_be_inserted[i+1],valid_next_points)
        valid_points.remove(new_point)
        new_path.append(new_point)
    new_path.append(target_point)
    if visualize:
        print("Checking validity of new path: " , new_path)
    # new_path = douglas_peucker(new_path)
    for i in range(len(new_path)-1):
        first_point = new_path[i]
        second_point = new_path[i+1]
        points_in_between = get_points_in_between(first_point, second_point)
        if i!= 0:
            points_in_between.pop(0)
        for point in points_in_between:
            if(is_valid(point, depth,grid,drone_occupancy_copy)):
                x, y, z = point
                drone_occupancy_copy[x][y][z].append((index_path_to_be_tweaked + 1, depth))
                depth += 1
            else:
                if visualize:
                    print("Could not crossover")
                return [],[]
    return new_path,drone_occupancy_copy
    
def contains_tag(drone_occupancy,drone_tag):
    for i in range(len(drone_occupancy)):
        for j in range(len(drone_occupancy)):
            for k in range(len(drone_occupancy)):
                for l in range(len(drone_occupancy[i][j][k])):
                    if(drone_occupancy[i][j][k][l][0] == drone_tag):
                        return True
    return False

def tweak_path(drone_paths, index_path_to_be_tweaked, drone_occupancy,starting_point,target_point, grid, visualize = False):
    if visualize:
        print("Tweaking path for drone " + str(index_path_to_be_tweaked + 1))
    
    old_path = drone_paths[index_path_to_be_tweaked]
    drone_occupancy_copy = drone_occupancy.copy()
    drone_occupancy_copy = remove_from_occurence(old_path,drone_occupancy_copy,index_path_to_be_tweaked+1)
    new_path = []
    new_path.append(starting_point)
    valid_points = get_valid_points(grid,starting_point,target_point)
    depth = 0
    end = len(old_path)-2
    for i in range(end):
        valid_next_points, point_depths = get_all_valid_next_points(grid,new_path[i],valid_points,drone_occupancy,depth)
        if(i == end - 1):
            previous_valid_points = get_all_valid_previous_points(grid,target_point,valid_next_points,drone_occupancy,point_depths)
            valid_next_points = previous_valid_points
        if(old_path[i+1] in valid_next_points):
            valid_next_points.remove(old_path[i+1])
        if(len(valid_next_points) == 0):
            if visualize:
                print("Path cannot be tweaked")
            return [],[]
        new_point = get_closest_valid_point(old_path[i+1],valid_next_points)
        valid_points.remove(new_point)
        new_path.append(new_point)
    new_path.append(target_point)
    # new_path = douglas_peucker(new_path)
    if visualize:
        print("Checking validity of new path: " , new_path)
    for i in range(len(new_path)-1):
        first_point = new_path[i]
        second_point = new_path[i+1]
        points_in_between = get_points_in_between(first_point, second_point)
        if(i!= 0):
            points_in_between.pop(0)
        # print("Points in between: ", points_in_between)
        for point in points_in_between:
            if(is_valid(point, depth,grid,drone_occupancy_copy)):
                x, y, z = point
                # print("Valid point: ", point, "Depth: ", depth)
                drone_occupancy_copy[x][y][z].append((index_path_to_be_tweaked + 1, depth))
                depth += 1
            else:
                print("Im here")
                return [],[]
    return new_path,drone_occupancy_copy





# def new_check_feasibility(drone_paths,grid):
#     print("Checking feasibility")
#     drone_occupancy = [[[ [] for _ in range(len(grid))] for _ in range(len(grid))] for _ in range(len(grid))]
#     new_drone_paths = []
#     i=0
#     while i < len(drone_paths):
#         path = drone_paths[i]
#         valid = True
#         drone_occupancy_copy = drone_occupancy.copy()
#         new_path = []
#         valid_points = get_valid_points(grid,path[0],path[-1])
#         depth = 0
#         for i in range(len(path) - 1):
#             pathCounter = 0
#             first_point = path[i]
#             second_point = path[i+1]
#             next_valid_points = []
#             if(not valid):
#                 valid = True
#                 print("Replacing Faulty point")
#                 if(len(next_valid_points)==0):
#                     next_valid_points = get_all_valid_next_points(grid,first_point,valid_points,drone_occupancy_copy,depth)
#                 second_point = get_closest_valid_point(path[i+1],next_valid_points) 
#                 next_valid_points.remove(second_point)
#             points_in_between = get_points_in_between(first_point, second_point)
#             valid_points_copy = valid_points.copy()
#             for point in points_in_between:
#                 if(is_valid(point, depth,grid,drone_occupancy_copy) and point in valid_points):
#                     valid_points.remove(point)
#                     next_valid_points = []
#                     drone_occupancy_copy[point[0]][point[1]][point[2]].append((i+1,depth))
#                     depth += 1
#                 else:
#                     print("Invalid point")
#                     if(len(next_valid_points)>0 or pathCounter == 0):
#                         pathCounter+=1
#                         valid = False
#                     else:
#                         return []
#                     break
#             if(valid):
#                 new_path.append(first_point)
#                 valid_points = valid_points_copy
#                 drone_occupancy = drone_occupancy_copy
#                 i+=1
#         new_drone_paths.append(new_path)
#     return new_drone_paths


def build_grid(obstacles, size_of_grid):
    """
    Args:
        obstacles (list): List of obstacle representations.
        size_of_grid (int): Size of the 3D grid representation of the environment.
    Returns:
        list: A 3D grid representation of the environment.
    Builds a grid representation of the environment.
    0: Free space
    1: Obstacle
    """ 
    grid = [[[0 for _ in range(size_of_grid)] for _ in range(size_of_grid)] for _ in range(size_of_grid)]
    # Set all obstacles to 1
    for obstacle in obstacles:
        for i, j in itertools.product(range(obstacle[0][0], obstacle[1][0] + separation + 1 ), range(obstacle[0][1], obstacle[1][1]  + separation + 1)):
            for k in range(obstacle[0][2], obstacle[1][2] + separation + 1) :
                grid[i][j][k] = 1
    return grid


# def get_single_path_with_bfs(starting_point, target_point, grid, drone_occupancy):
#     """
#     Get a path from the start point to the target point using BFS.

#     Args:
#         starting_point (tuple): The start point (x, y, z).
#         target_point (tuple): The target point (x, y, z).
#         grid (list): A 3D grid representation of the obstacle placement.
#         drone_occupancy (list): A 3D grid representation of the drones placement.

#     Returns:
#         list: A list of control points defining the path.
#     """

#     def is_valid(parent, point, layer):
#         x, y, z = point
#         admissible = 0 <= x < len(grid) and 0 <= y < len(grid) and 0 <= z < len(grid) and grid[x][y][z] == 0
#         if(not admissible):
#            return False
#         current_drones_occupying_cell = drone_occupancy[x][y][z]
#         for _ , current_drone_depth in current_drones_occupying_cell:
#             if layer[parent] + 1 == current_drone_depth:
#                 return False
#         return admissible

#     def get_neighbors(point,layer):
#         x, y, z = point
#         neighbors = [(x + dx, y + dy, z + dz) for dx in [-1, 0, 1] for dy in [-1, 0, 1] for dz in [-1, 0, 1] if
#                      (dx != 0 or dy != 0 or dz != 0)]
#         return [neighbor for neighbor in neighbors if is_valid(point, neighbor,layer)]

#     start = tuple(map(int, starting_point))
#     target = tuple(map(int, target_point))
#     queue = Queue()
#     queue.put(start)
#     came_from = {}
#     came_from[start] = None
#     layer = {}
#     layer[start] = 2


#     while not queue.empty():
#         current = queue.get()

#         if current == target:
#             path = [current]
#             while current != start:
#                 current = came_from[current]
#                 path.insert(0, current)
#             return path

#         for neighbor in get_neighbors(current,layer):
#             if neighbor not in came_from:
#                 queue.put(neighbor)
#                 came_from[neighbor] = current   
#                 layer[neighbor] = layer[current] + 1
#     return []


# def get_all_paths_with_bfs(starting_points, target_points, grid):
#     """
#     Get all paths from the start points to the target points using BFS.

#     Args:
#         starting_points (list): A list of he start points (x, y, z).
#         target_points (list): A list of the target points (x, y, z).
#         grid (list): A 3D grid representation of the environment.

#     Returns:
#         list: A list of paths, where each path is a list of control points.
#     """
#     all_paths = []
#     simplified_paths = []
#     drone_occupancy = [[[ [] for _ in range(len(grid))] for _ in range(len(grid))] for _ in range(len(grid))]
#     for drone_tag, (starting_point, target_point) in enumerate(zip(starting_points, target_points), start=1):
#         drone_occupancy[starting_point[0]][starting_point[1]][starting_point[2]].append((drone_tag +1, 1))
#         drone_occupancy[target_point[0]][target_point[1]][target_point[2]].append((drone_tag +1, 100))

#     for drone_tag, (starting_point, target_point) in enumerate(zip(starting_points, target_points), start=1):
#         path = get_single_path_with_bfs(starting_point, target_point, grid, drone_occupancy)
        
#         depth = 2
#         for point in path:
#             if point in [starting_point, target_point]:
#                 continue
#             x, y, z = point
#             drone_occupancy[x][y][z].append((drone_tag, depth))
#             depth += 1

#         simplified_path = douglas_peucker(path)
#         all_paths.append(path)
#         simplified_paths.append(simplified_path)

        
#     return simplified_paths, drone_occupancy

def generate_initial_solution(size_of_grid, starting_points, target_points, obstacles):
    """
    Generate an initial solution to the problem using BFS.

    Args:
        starting_points (list): A list of he start points (x, y, z).
        target_points (list): A list of the target points (x, y, z).
        grid (list): A 3D grid representation of the environment.
    Returns:
        list: Initial solution.
    """
    grid = build_grid(obstacles, size_of_grid)
    paths, drone_occupancy = generate_initial_paths(starting_points, target_points, grid)

    return paths, grid , drone_occupancy

def generate_initial_paths(starting_points, target_points, grid):
    '''
    Generate initial random paths for all drones
    starting_points (list): A list of the start points (x, y, z).
    target_points (list): A list of the target points (x, y, z).
    grid (list): A 3D grid representation of the environment.
    '''
    paths = []
    drone_occupancy = [[[ [] for _ in range(len(grid))] for _ in range(len(grid))] for _ in range(len(grid))]
    drone_tag = 1
    for starting_point, target_point in zip(starting_points, target_points):
        # print("Generating path number: ", drone_tag)
        path,new_drone_occupancy = generate_path(starting_point, target_point, grid,drone_tag, drone_occupancy)
        drone_occupancy = new_drone_occupancy
        drone_tag += 1
        # path = douglas_peucker(path)
        paths.append(path)
    return paths,drone_occupancy

def get_valid_points(grid,starting_point,target_point):
    min_x = min(starting_point[0], target_point[0])
    max_x = max(starting_point[0], target_point[0])
    min_y = min(starting_point[1], target_point[1])
    max_y = max(starting_point[1], target_point[1])
    min_z = min(starting_point[2], target_point[2])
    max_z = max(starting_point[2], target_point[2])

    valid_points = []
    for x in range(min_x, max_x + 1):
        for y in range(min_y, max_y + 1):
            for z in range(min_z, max_z + 1):
                if grid[x][y][z] == 0 and (x, y, z) != starting_point and (x, y, z) != target_point:
                    valid_points.append((x, y, z))
    return valid_points

def get_all_valid_next_points(grid,starting_point,points,drone_occupancy,initial_depth):
    valid_points = []
    point_depths = []
    for point in points:
        current_depth = initial_depth
        points_in_between = get_points_in_between(starting_point, point)
        rejected_Flag = False
        for inner_point in points_in_between:
            if(grid[inner_point[0]][inner_point[1]][inner_point[2]]==1):
                rejected_Flag = True
                break
            drones_in_cell = drone_occupancy[inner_point[0]][inner_point[1]][inner_point[2]]
            for _, depth in drones_in_cell:
                if(current_depth == depth):
                    rejected_Flag = True
                    break
            if(rejected_Flag):
                break
            current_depth+=1
        if(rejected_Flag):
            continue
        valid_points.append(point)
        point_depths.append(current_depth)
    return valid_points, point_depths

def get_all_valid_previous_points(grid,target_point,points,drone_occupancy,initial_depths):
    valid_points = []
    for i,point in enumerate(points):
        current_depth = initial_depths[i]
        points_in_between = get_points_in_between(point, target_point)
        rejected_Flag = False
        for inner_point in points_in_between:
            if(grid[inner_point[0]][inner_point[1]][inner_point[2]]==1):
                rejected_Flag = True
                break
            drones_in_cell = drone_occupancy[inner_point[0]][inner_point[1]][inner_point[2]]
            for _, depth in drones_in_cell:
                if(current_depth == depth):
                    rejected_Flag = True
                    break
            if(rejected_Flag):
                break
            current_depth+=1
        if(rejected_Flag):
            continue
        valid_points.append(point)
    return valid_points
        
def is_valid(point, depth,grid,drone_occupancy):
    x, y, z = point
    # print("Checking Validity of point: " , point)
    admissible = 0 <= x < len(grid) and 0 <= y < len(grid) and 0 <= z < len(grid) and grid[x][y][z] == 0
    if(not admissible):
        print(point, " Not admissible")
        return False
    current_drones_occupying_cell = drone_occupancy[x][y][z]
    for drone_tag , current_drone_depth in current_drones_occupying_cell:
        if depth == current_drone_depth:
            print(point, " Already occupied by drone ", drone_tag)
            return False
    return admissible

def generate_path(starting_point, target_point, grid,drone_tag, drone_occupancy):
    '''
    Generate initial random path for a single drone
    starting_point (tuple): The start point (x, y, z).
    target_point (tuple): The target point (x, y, z).
    '''
    break_outer_loop = False
    path = []
    drone_occupancy_copy = []
    depth = 0
    valid_points = get_valid_points(grid,starting_point,target_point)
    path_found = False
    while len(valid_points) > 0:
        path = []
        path.append(starting_point)
        current_point = starting_point
        drone_occupancy_copy = drone_occupancy.copy()
        num_control_points = random.randint(min_number_control_points,max_number_control_points)
        for i in range(num_control_points):
            next_valid_points, point_depths = get_all_valid_next_points(grid,current_point,valid_points,drone_occupancy_copy,depth)
            if(i == num_control_points - 1):
                previous_valid_points = get_all_valid_previous_points(grid,target_point,next_valid_points,drone_occupancy,point_depths)
                next_valid_points = previous_valid_points
            point = random.choice(next_valid_points)
            valid_points.remove(point)
            current_point = point
            path.append(point)
        path.append(target_point)
        depth = 0
        i = 0
        for i in range(len(path) - 1):
            if break_outer_loop:
                break_outer_loop = False
                break
            first_point = path[i]
            second_point = path[i+1]
            points_in_between = get_points_in_between(first_point, second_point)
            for point in points_in_between:
                if(is_valid(point, depth,grid,drone_occupancy_copy)):
                    drone_occupancy_copy[point[0]][point[1]][point[2]].append((drone_tag,depth))
                    depth += 1
                else:
                    break_outer_loop = True
                    break
        if i == len(path) - 2:
            # print("Path found")
            path_found = True
            break
    if path_found:
        return path, drone_occupancy_copy
    else:
        # print("Path not found")
        return [], []


    
    







def douglas_peucker(points):
    """
    Applys douglas_peucker algorithm on generated points from bfs to lower the number of control points

    Args:
        points (list): A list of all paths generated by BFS.

    Returns:
        list: similar path with fewer number of control points.
    """
    if len(points) < 2:
        return [points[0], points[1]]
    
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

    if max_distance <= tolerance:
        return [start, end]
    # Recursively simplify the path
    first_segment = douglas_peucker(points[:max_index + 1])
    second_segment = douglas_peucker(points[max_index:])

    return first_segment[:-1] + second_segment  # Exclude the duplicated point
