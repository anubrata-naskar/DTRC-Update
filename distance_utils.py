"""
Distance calculation utilities including Google Maps API and Haversine distance
"""
import math
import requests
import time
import numpy as np
import pandas as pd
from file_utils import GOOGLE_MAPS_API_KEY


def get_google_maps_distance(lat1, lon1, lat2, lon2):
    url = f"https://maps.googleapis.com/maps/api/directions/json?origin={lat1},{lon1}&destination={lat2},{lon2}&mode=driving&key={GOOGLE_MAPS_API_KEY}"
    
    try:
        response = requests.get(url)
        data = response.json()
        
        if data["status"] == "OK":
            distance_m = data["routes"][0]["legs"][0]["distance"]["value"]
            distance_km = distance_m / 1000.0
            return distance_km
        else:
            print(f"Google Maps API request failed with status: {data['status']}")
            return haversine_distance(lat1, lon1, lat2, lon2)
            
    except Exception as e:
        print(f"Error accessing Google Maps API: {e}")
        return haversine_distance(lat1, lon1, lat2, lon2)


def distance_matrix_from_xy(lat_coordinates, lon_coordinates, use_google_maps=True):
    n = len(lat_coordinates)
    dist_matrix = np.zeros((n, n))
    
    distance_cache = {}

    for i in range(n):
        for j in range(i + 1, n):
            cache_key = f"{lat_coordinates[i]},{lon_coordinates[i]}-{lat_coordinates[j]},{lon_coordinates[j]}"
            
            if use_google_maps:
                if cache_key in distance_cache:
                    dist = distance_cache[cache_key]
                else:
                    dist = get_google_maps_distance(
                        lat_coordinates[i], lon_coordinates[i], 
                        lat_coordinates[j], lon_coordinates[j]
                    )
                    distance_cache[cache_key] = dist
                    time.sleep(0.2)
            else:
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
