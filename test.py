import math


def points_between_two_points(point1, point2):
    points = []
    num_points = math.max(abs(point1[0] - point2[0]), abs(point1[1] - point2[1]), abs(point1[2] - point2[2]))
    for t in range(num_points + 1):
        t_normalized = t / num_points
        x = int(point1[0] + t_normalized * (point2[0] - point1[0]))
        y = int(point1[1] + t_normalized * (point2[1] - point1[1]))
        z = int(point1[2] + t_normalized * (point2[2] - point1[2]))
        points.append((x, y, z))
    return points

# Example usage:
point_A = (1, 2, 3)
point_B = (4, 7, 11)
num_points = 8

generated_points = points_between_two_points(point_A, point_B, num_points)

print("Generated Points:", generated_points)
