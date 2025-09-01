"""
Distance calculation utilities using Euclidean distance
"""
import numpy as np
import pandas as pd


def euclidean_distance(lat1, lon1, lat2, lon2):
    """
    Calculate Euclidean distance between two points
    """
    return ((lat2 - lat1) ** 2 + (lon2 - lon1) ** 2) ** 0.5


def project_point_to_segment(p_lat, p_lon, s_lat, s_lon, e_lat, e_lon):
    """
    Project a point onto a line segment and return the parametric position (0 to 1)
    """
    a = e_lat - s_lat
    b = e_lon - s_lon
    
    if a == 0 and b == 0:
        return 0
    
    t = ((p_lat - s_lat) * a + (p_lon - s_lon) * b) / (a * a + b * b)
    return t


def distance_matrix_from_xy(lat_coordinates, lon_coordinates, use_google_maps=False):
    """
    Create distance matrix from coordinates using Euclidean distance
    The use_google_maps parameter is kept for compatibility but ignored
    """
    n = len(lat_coordinates)
    dist_matrix = np.zeros((n, n))
    
    print(f"Building distance matrix for {n} nodes using Euclidean distance...")
    
    for i in range(n):
        for j in range(i + 1, n):
            dist = euclidean_distance(
                lat_coordinates[i], lon_coordinates[i],
                lat_coordinates[j], lon_coordinates[j]
            )
            dist_matrix[i][j] = dist
            dist_matrix[j][i] = dist

    return pd.DataFrame(dist_matrix)
