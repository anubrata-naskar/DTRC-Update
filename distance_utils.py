"""
Distance calculation utilities including Google Maps API with caching and Haversine distance
"""
import math
import requests
import time
import numpy as np
import pandas as pd
from file_utils import GOOGLE_MAPS_API_KEY


def get_google_maps_distance(lat1, lon1, lat2, lon2):
    """
    Get Google Maps distance (deprecated - use cached version)
    This function is kept for backward compatibility
    """
    from google_maps_cache import get_cache_instance
    cache = get_cache_instance()
    return cache.get_distance(lat1, lon1, lat2, lon2)


def distance_matrix_from_xy(lat_coordinates, lon_coordinates, use_google_maps=True):
    n = len(lat_coordinates)
    dist_matrix = np.zeros((n, n))
    
    if use_google_maps:
        # Initialize cache for this coordinate set
        from google_maps_cache import get_cache_instance
        cache = get_cache_instance()
        cache.initialize_for_coordinates(lat_coordinates, lon_coordinates)
        
        print(f"Building distance matrix for {n} nodes using Google Maps API with caching...")
        api_calls_made = 0
        cache_hits = 0
        
        for i in range(n):
            for j in range(i + 1, n):
                # Check if this distance was in cache
                cache_key = cache._create_cache_key(
                    lat_coordinates[i], lon_coordinates[i],
                    lat_coordinates[j], lon_coordinates[j]
                )
                was_cached = cache_key in cache.cache_data
                
                # Get distance (will use cache if available)
                dist = cache.get_distance(
                    lat_coordinates[i], lon_coordinates[i], 
                    lat_coordinates[j], lon_coordinates[j]
                )
                
                if was_cached:
                    cache_hits += 1
                else:
                    api_calls_made += 1
                
                dist_matrix[i][j] = dist
                dist_matrix[j][i] = dist
        
        # Save cache and show statistics
        cache.finalize_cache()
        
        print(f"\n=== Distance Matrix Statistics ===")
        print(f"Total distance pairs: {(n * (n-1)) // 2}")
        print(f"Cache hits: {cache_hits}")
        print(f"New API calls: {api_calls_made}")
        print(f"API cost saved: {cache_hits} calls")
        print("==================================\n")
    
    else:
        # Use Haversine distance
        print(f"Building distance matrix for {n} nodes using Haversine distance...")
        for i in range(n):
            for j in range(i + 1, n):
                dist = haversine_distance(
                    lat_coordinates[i], lon_coordinates[i],
                    lat_coordinates[j], lon_coordinates[j]
                )
                dist_matrix[i][j] = dist
                dist_matrix[j][i] = dist

    return pd.DataFrame(dist_matrix)


def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371.0
    
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    dlon = lon2_rad - lon1_rad
    dlat = lat2_rad - lat1_rad
    
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    distance = R * c
    return distance


def project_point_to_segment(p_lat, p_lon, s_lat, s_lon, e_lat, e_lon):
    a = e_lat - s_lat
    b = e_lon - s_lon
    
    if a == 0 and b == 0:
        return 0
    
    t = ((p_lat - s_lat) * a + (p_lon - s_lon) * b) / (a * a + b * b)
    return t
