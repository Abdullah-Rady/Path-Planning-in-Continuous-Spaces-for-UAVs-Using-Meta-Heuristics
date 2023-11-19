import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import json
import time
import vtk

def get_paths(data, drone_number):
    drone_paths = {}
    drone_paths = []
    for j in range(len(data)):
        for k in range(len(data[j])):
            for l in range(len(data[j][k])):
                drones_in_cell = data[j][k][l]
                for drone_tag,time_stamp in drones_in_cell:
                    if drone_tag == drone_number:
                        drone_paths.append((j,k,l,time_stamp))
    #sort drone_paths[i] by time_stamp
    drone_paths.sort(key=lambda tup: tup[3])
    return drone_paths

def visualize_problem_solution(ps_list, pt_list, obstacle_list, solution_paths=None):
    # Create a VTK renderer
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(1, 1, 1)  # Set background color (white)
    

    # Create a VTK render window
    render_window = vtk.vtkRenderWindow()
    render_window.SetWindowName("3D Grid with Target Points and Obstacles")
    render_window.SetSize(800, 600)  # Set initial size of the window
    render_window.AddRenderer(renderer)
    render_window_interactor = vtk.vtkRenderWindowInteractor()
    render_window_interactor.SetRenderWindow(render_window)

    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(render_window)

    # Create a grid
    grid_source = vtk.vtkPlaneSource()
    grid_source.SetResolution(10, 10)  # Set the resolution of the grid
    grid_mapper = vtk.vtkPolyDataMapper()
    grid_mapper.SetInputConnection(grid_source.GetOutputPort())

    grid_actor = vtk.vtkActor()
    grid_actor.SetMapper(grid_mapper)
    grid_actor.GetProperty().SetColor(0.8, 0.8, 0.8)  # Set grid color to light gray

    # Add axes to visualize orientation
    axes = vtk.vtkAxesActor()
    style = vtk.vtkInteractorStyleTrackballCamera()
    interactor.SetInteractorStyle(style)

    # Create a widget to show the orientation marker
    orientation_widget = vtk.vtkOrientationMarkerWidget()
    orientation_widget.SetOrientationMarker(axes)
    orientation_widget.SetInteractor(interactor)
    orientation_widget.SetViewport(0.8, 0, 1.0, 0.2) 



    # Create point data for start points (ps_list)
    start_points = vtk.vtkPoints()
    for point in ps_list:
        start_points.InsertNextPoint(point)

    # Create a polydata to hold the points
    start_polydata = vtk.vtkPolyData()
    start_polydata.SetPoints(start_points)

    # Create a vertex filter to render points as vertices
    vertex_filter = vtk.vtkVertexGlyphFilter()
    vertex_filter.SetInputData(start_polydata)

    # Create a mapper
    start_mapper = vtk.vtkPolyDataMapper()
    start_mapper.SetInputConnection(vertex_filter.GetOutputPort())

    # Create an actor
    start_actor = vtk.vtkActor()
    start_actor.SetMapper(start_mapper)
    start_actor.GetProperty().SetColor(0, 0, 1)  # Set color to blue
    start_actor.GetProperty().SetPointSize(10)  # Set point size

    # Similar steps for target points (pt_list), obstacles, and solution paths...

    target_points = vtk.vtkPoints()
    for point in pt_list:
        target_points.InsertNextPoint(point)

    # Create a polydata to hold the target points
    target_polydata = vtk.vtkPolyData()
    target_polydata.SetPoints(target_points)

    # Create a vertex filter to render points as vertices
    target_vertex_filter = vtk.vtkVertexGlyphFilter()
    target_vertex_filter.SetInputData(target_polydata)

    # Create a mapper for target points
    target_mapper = vtk.vtkPolyDataMapper()
    target_mapper.SetInputConnection(target_vertex_filter.GetOutputPort())

    # Create an actor for target points
    target_actor = vtk.vtkActor()
    target_actor.SetMapper(target_mapper)
    target_actor.GetProperty().SetColor(0, 1, 0)  # Set color to green
    target_actor.GetProperty().SetPointSize(10)  # Set point size

    obstacle_points = vtk.vtkPoints()
    for obstacle in obstacle_list:
        for x_value in range(obstacle[0][0], obstacle[1][0] + 1):
            for y_value in range(obstacle[0][1], obstacle[1][1] + 1):
                for z_value in range(obstacle[0][2], obstacle[1][2] + 1):
                    obstacle_points.InsertNextPoint(x_value, y_value, z_value)

    # Create a polydata to hold the obstacle points
    obstacle_polydata = vtk.vtkPolyData()
    obstacle_polydata.SetPoints(obstacle_points)

    # Create a vertex filter to render points as vertices for obstacles
    obstacle_vertex_filter = vtk.vtkVertexGlyphFilter()
    obstacle_vertex_filter.SetInputData(obstacle_polydata)

    # Create a mapper for obstacles
    obstacle_mapper = vtk.vtkPolyDataMapper()
    obstacle_mapper.SetInputConnection(obstacle_vertex_filter.GetOutputPort())

    # Create an actor for obstacles
    obstacle_actor = vtk.vtkActor()
    obstacle_actor.SetMapper(obstacle_mapper)
    obstacle_actor.GetProperty().SetColor(1, 0, 0)  # Set color to red
    obstacle_actor.GetProperty().SetPointSize(10)  # Set point size

    # Create a polydata for solution paths (assuming solution_paths is a list of paths)
    if solution_paths != None:
        solution_paths_polydata = vtk.vtkPolyData()
        points = vtk.vtkPoints()

        lines = vtk.vtkCellArray()  # Create lines to represent paths

        for path in solution_paths:
            line = vtk.vtkPolyLine()
            for point in path:
                points.InsertNextPoint(point)
                line.GetPointIds().InsertNextId(points.GetNumberOfPoints() - 1)  # Insert point IDs for the line

            lines.InsertNextCell(line)

        solution_paths_polydata.SetPoints(points)
        solution_paths_polydata.SetLines(lines)

        # Create a mapper for solution paths
        solution_paths_mapper = vtk.vtkPolyDataMapper()
        solution_paths_mapper.SetInputData(solution_paths_polydata)

        # Create an actor for solution paths
        solution_paths_actor = vtk.vtkActor()
        solution_paths_actor.SetMapper(solution_paths_mapper)
        solution_paths_actor.GetProperty().SetColor(0, 0, 0)  # Set color to black
        solution_paths_actor.GetProperty().SetLineWidth(2)  # Set line width


    # Add actors for obstacles and solution paths to the renderer
    renderer.AddActor(obstacle_actor)

    if solution_paths != None:
        renderer.AddActor(solution_paths_actor)
    # Add actors for target points, obstacles, and paths to the renderer
    renderer.AddActor(target_actor)

    # Add actors to the renderer
    renderer.AddActor(start_actor)
    renderer.AddActor(grid_actor)


    # Set camera position and orientation
    renderer.GetActiveCamera().SetPosition(10, 10, 10)
    renderer.GetActiveCamera().SetFocalPoint(0, 0, 0)
    renderer.GetActiveCamera().Azimuth(30)
    renderer.GetActiveCamera().Elevation(30)
    renderer.ResetCamera()

    # Start the visualization
    render_window.Render()
    render_window_interactor.Start()
    interactor.Start()


# def visualize_problem_solution(ps_list, pt_list, obstacle_list, solution_paths):
#     # Function to plot points
#     def plot_points(points, ax, color, marker):
#         for point in points:
#             ax.scatter(*point, c=color, marker=marker, s=100)

#     # Function to plot lines between start and end points
#     def plot_path(path, ax, color):
#         for i in range(0, len(path) - 1):
#             ax.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], [path[i][2], path[i + 1][2]], c='black', linewidth=1)
#             # ax.plot(path[i][0], path[i][1], path[i][1], path[i + 1][2]], c=color, linewidth=1, marker='_')


#     # Function to plot obstacles
#     def plot_obstacles(obstacles, ax):
#         for obstacle in obstacles:
#             starting_point = obstacle[0]
#             ending_point = obstacle[1]
#             obstacle_points = []
#             for x_value in range(starting_point[0], ending_point[0] + 1):
#                 for y_value in range(starting_point[1], ending_point[1] + 1):
#                     for z_value in range(starting_point[2], ending_point[2] + 1):
#                         obstacle_points.append((x_value, y_value, z_value))
#             plot_points(obstacle_points, ax, 'red', 's')

#     # Create a 3D plot
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')

#     # Plot the grid
#     # plot_grid(size_of_grid1, ax)


#     # Plot lines connecting start and end points
#     # plot_lines(ps_list, pt_list, ax)

#     plot_points(ps_list, ax, 'blue', 'x')  # Plot start points
#     # Plot target points
#     plot_points(pt_list, ax, 'green', 'o')

#     # Plot obstacles
#     plot_obstacles(obstacle_list, ax)

#     colors = ['#FF5733', '#33FF57', '#3366FF', '#FF33DD', '#33FFFF', '#FF33CC', '#6633FF', '#33FFDD', '#FF3366', '#33CCFF']

#     if solution_paths is not None:
#         for i, path in enumerate(solution_paths):
#                 plot_path(path, ax, colors[i])

#     # Set labels
#     ax.set_xlabel('X-axis')
#     ax.set_ylabel('Y-axis')
#     ax.set_zlabel('Z-axis')

#     # Set title
#     plt.title('3D Grid with Target Points and Obstacles')

#     # Show the plot
#     plt.show()

# def visualize_problem_solution(ps_list, pt_list, obstacle_list, solution_paths):
#     lines = []
#     for path in solution_paths:
#         points = np.array(path)
#         line = p3j.Line(geometry=p3j.BufferGeometry(attributes={'position': p3j.BufferAttribute(array=points)}),
#                      material=p3j.LineBasicMaterial(color='blue'))
#         lines.append(line)

#     # Create a scatter plot for points
#     scatter_points = []
#     for point in ps_list:
#         scatter = p3j.Mesh(geometry=p3j.SphereGeometry(radius=0.5), material=p3j.MeshBasicMaterial(color='red'), position=point)
#         scatter_points.append(scatter)

#     # Create a scatter plot for target points
#     scatter_target = []
#     for point in pt_list:
#         scatter = p3j.Mesh(geometry=p3j.SphereGeometry(radius=0.5), material=p3j.MeshBasicMaterial(color='green'), position=point)
#         scatter_target.append(scatter)

#     scatter_obstacles = []
#     for obstacle in obstacle_list:
#         # Extract x, y, z coordinates from the obstacle list
#         x, y, z = zip(*[(i, j, k) for i in range(obstacle[0][0], obstacle[1][0] + 1)
#                         for j in range(obstacle[0][1], obstacle[1][1] + 1)
#                         for k in range(obstacle[0][2], obstacle[1][2] + 1)])
        
#         # Create a list of positions for each point in the obstacle
#         positions = list(zip(map(float, x), map(float, y), map(float, z)))        
#         # Create a mesh for each position in the obstacle
#         for pos in positions:
#             scatter = p3j.Mesh(geometry=p3j.BoxGeometry(width=1, height=1, depth=1),
#                             material=p3j.MeshBasicMaterial(color='red'),
#                             position=pos)  # Assign each position individually
#             scatter_obstacles.append(scatter)

#     # Create a scene
#     scene = p3j.Scene(children=lines + scatter_points + scatter_target + scatter_obstacles)

#     # Create a renderer
#     renderer = p3j.Renderer(camera=p3j.PerspectiveCamera(position=[10, 10, 10], aspect=1),
#                         scene=scene,
#                         controls=[p3j.OrbitControls(controlling=p3j.PerspectiveCamera())])

#     # Display the renderer
#     renderer


def plot_fitness_over_iterations( fitness_values):
    """
    Plot the best fitness over iterations.

    Parameters:
    - fitness_values: List of corresponding best fitness values.
    """
    window_size = 10
    mean_per_population, std_per_population = calculate_stats_per_window(fitness_values, window_size)
    plt.plot(np.arange(1, len(fitness_values) // window_size + 1), mean_per_population, marker='o', linestyle='-')
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
    generations = np.arange(1, len(fitness_values) // window_size + 1)
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
    mean_per_population, std_per_population = calculate_stats_per_window(fitness_values, 10)

    fitness_mean = np.mean(mean_per_population)
    fitness_std = np.std(std_per_population)
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

