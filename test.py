import numpy as np

def calculate_stats_per_window(fitness_values, window_size):

    stats_per_window = []
    num_windows = len(fitness_values) // window_size

    for i in range(num_windows):
        window_start = i * window_size
        window_end = window_start + window_size
        window_fitness = fitness_values[window_start:window_end]
        window_mean = np.mean(window_fitness)
        stats_per_window.append(window_mean)
        
    return stats_per_window

