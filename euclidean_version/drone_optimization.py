"""
Drone takeoff and landing point optimization utilities
"""
import numpy as np
from scipy.optimize import minimize
from distance_utils import euclidean_distance
from file_utils import dynamic_battery_model
from drone_validation import validate_drone_virtual_points


def optimize_drone_takeoff_landing(truck_routes, drone_routes, distances_df, lat_coords, lon_coords, demands):
    from scipy.optimize import minimize
    
    optimized_drone_routes = []
    
    for truck_idx, truck_route in enumerate(truck_routes):
        truck_drone_routes = []
        
        truck_segments = []
        for i in range(len(truck_route) - 1):
            start_node = truck_route[i]
            end_node = truck_route[i+1]
            truck_segments.append((
                start_node, end_node,
                lat_coords[start_node], lon_coords[start_node],
                lat_coords[end_node], lon_coords[end_node]
            ))
        
        for drone_idx, drone_route_list in enumerate(drone_routes[truck_idx]):
            optimized_drone_route_list = []
            
            for route in drone_route_list:
                if not route:
                    optimized_drone_route_list.append([])
                    continue
                
                takeoff = route[0]
                delivery_nodes = route[1:-1]
                landing = route[-1]
                
                if isinstance(takeoff, str) and takeoff.startswith('V('):
                    for segment_idx, segment in enumerate(truck_segments):
                        start_node, end_node, s_lat, s_lon, e_lat, e_lon = segment
                        
                        def objective(t):
                            proj_lat = s_lat + t * (e_lat - s_lat)
                            proj_lon = s_lon + t * (e_lon - s_lon)
                            
                            total_dist = 0
                            for node in delivery_nodes:
                                if isinstance(node, str):
                                    continue
                                dist = euclidean_distance(proj_lat, proj_lon, lat_coords[node], lon_coords[node])
                                total_dist += dist
                            
                            return total_dist
                        
                        res = minimize(objective, 0.5, bounds=[(0, 1)])
                        t_opt = res.x[0]
                        
                        proj_lat = s_lat + t_opt * (e_lat - s_lat)
                        proj_lon = s_lon + t_opt * (e_lon - s_lon)
                        
                        optimized_takeoff = f"V({proj_lat:.6f},{proj_lon:.6f})"
                        takeoff = optimized_takeoff
                        break
                
                if isinstance(landing, str) and landing.startswith('V('):
                    for segment_idx, segment in enumerate(truck_segments):
                        start_node, end_node, s_lat, s_lon, e_lat, e_lon = segment
                        
                        def objective(t):
                            proj_lat = s_lat + t * (e_lat - s_lat)
                            proj_lon = s_lon + t * (e_lon - s_lon)
                            
                            total_dist = 0
                            for node in delivery_nodes:
                                if isinstance(node, str):
                                    continue
                                dist = euclidean_distance(lat_coords[node], lon_coords[node], proj_lat, proj_lon)
                                total_dist += dist
                            
                            return total_dist
                        
                        res = minimize(objective, 0.5, bounds=[(0, 1)])
                        t_opt = res.x[0]
                        
                        proj_lat = s_lat + t_opt * (e_lat - s_lat)
                        proj_lon = s_lon + t_opt * (e_lon - s_lon)
                        
                        optimized_landing = f"V({proj_lat:.6f},{proj_lon:.6f})"
                        landing = optimized_landing
                        break
                
                optimized_route = [takeoff]
                optimized_route.extend(delivery_nodes)
                optimized_route.append(landing)
                
                optimized_drone_route_list.append(optimized_route)
            
            truck_drone_routes.append(optimized_drone_route_list)
        
        optimized_drone_routes.append(truck_drone_routes)
    
    return optimized_drone_routes
