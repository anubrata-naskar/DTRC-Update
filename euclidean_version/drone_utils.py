"""
Utility functions for drone operations in Euclidean space
"""
from distance_utils import euclidean_distance


def calculate_drone_flight_time(route, lat_coords, lon_coords, drone_speed=1.5):
    """
    Calculate the flight time for a drone route
    """
    total_distance = 0
    prev_node = route[0]
    
    for next_node in route[1:]:
        if isinstance(prev_node, str) and prev_node.startswith('V('):
            coords = prev_node.strip('V()').split(',')
            if len(coords) == 2:
                prev_lat, prev_lon = float(coords[0]), float(coords[1])
                
                if isinstance(next_node, str) and next_node.startswith('V('):
                    coords = next_node.strip('V()').split(',')
                    next_lat, next_lon = float(coords[0]), float(coords[1])
                    segment_distance = euclidean_distance(prev_lat, prev_lon, next_lat, next_lon)
                else:
                    segment_distance = euclidean_distance(prev_lat, prev_lon, lat_coords[next_node], lon_coords[next_node])
            else:
                segment_distance = 0
        elif isinstance(next_node, str) and next_node.startswith('V('):
            coords = next_node.strip('V()').split(',')
            if len(coords) == 2:
                next_lat, next_lon = float(coords[0]), float(coords[1])
                segment_distance = euclidean_distance(lat_coords[prev_node], lon_coords[prev_node], next_lat, next_lon)
            else:
                segment_distance = 0
        else:
            segment_distance = euclidean_distance(lat_coords[prev_node], lon_coords[prev_node], lat_coords[next_node], lon_coords[next_node])
        
        total_distance += segment_distance
        prev_node = next_node
    
    return total_distance / drone_speed
