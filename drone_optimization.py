"""
Drone takeoff and landing point optimization utilities
"""
import numpy as np
from scipy.optimize import minimize
from distance_utils import haversine_distance
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
                i
            ))
        
        for drone_idx, drone_route_list in enumerate(drone_routes[truck_idx]):
            optimized_drone_route_list = []
            
            for route in drone_route_list:
                if not route or len(route) <= 2:
                    optimized_drone_route_list.append(route)
                    continue
                
                delivery_nodes = []
                for node in route[1:-1]:
                    if not isinstance(node, str):
                        delivery_nodes.append(node)
                
                if not delivery_nodes:
                    optimized_drone_route_list.append(route)
                    continue
                
                total_payload = sum(demands.get(node, 0) for node in delivery_nodes)
                
                best_takeoff_segment = None
                best_landing_segment = None
                best_params = None
                best_score = float('inf')
                
                for takeoff_seg_idx, takeoff_segment in enumerate(truck_segments):
                    max_landing_idx = min(takeoff_seg_idx + 5, len(truck_segments))
                    
                    for landing_seg_idx in range(takeoff_seg_idx, max_landing_idx):
                        landing_segment = truck_segments[landing_seg_idx]
                        
                        takeoff_start, takeoff_end, _ = takeoff_segment
                        landing_start, landing_end, _ = landing_segment
                        
                        def get_point_on_segment(t, start_node, end_node):
                            lat = lat_coords[start_node] + t * (lat_coords[end_node] - lat_coords[start_node])
                            lon = lon_coords[start_node] + t * (lon_coords[end_node] - lon_coords[start_node])
                            return lat, lon
                        
                        def objective(vars):
                            t1, t2 = vars
                            
                            takeoff_lat, takeoff_lon = get_point_on_segment(t1, takeoff_start, takeoff_end)
                            landing_lat, landing_lon = get_point_on_segment(t2, landing_start, landing_end)
                            
                            total_distance = 0
                            
                            first_node = delivery_nodes[0]
                            total_distance += haversine_distance(takeoff_lat, takeoff_lon, lat_coords[first_node], lon_coords[first_node])

                            for i in range(len(delivery_nodes) - 1):
                                node1 = delivery_nodes[i]
                                node2 = delivery_nodes[i + 1]
                                total_distance += haversine_distance(lat_coords[node1], lon_coords[node1], 
                                                                    lat_coords[node2], lon_coords[node2])
                            
                            last_node = delivery_nodes[-1]
                            total_distance += haversine_distance(lat_coords[last_node], lon_coords[last_node], landing_lat, landing_lon)
                            
                            return total_distance
                        
                        def constraint(vars):
                            t1, t2 = vars
                            
                            truck_time = 0
                            
                            takeoff_seg_length = distances_df.iloc[takeoff_start, takeoff_end]
                            truck_time += (1 - t1) * takeoff_seg_length
                            
                            for idx in range(takeoff_seg_idx + 1, landing_seg_idx):
                                seg_start, seg_end, _ = truck_segments[idx]
                                truck_time += distances_df.iloc[seg_start, seg_end]
                            
                            if takeoff_seg_idx != landing_seg_idx:
                                landing_seg_length = distances_df.iloc[landing_start, landing_end]
                                truck_time += t2 * landing_seg_length
                            else:
                                truck_time = (t2 - t1) * distances_df.iloc[takeoff_start, takeoff_end]
                            
                            drone_time = objective(vars) / 1.5
                            
                            return drone_time - truck_time
                        
                        bounds = [(0, 1), (0, 1)]
                        
                        if takeoff_seg_idx == landing_seg_idx:
                            initial_guess = [0.2, 0.8]
                        else:
                            initial_guess = [0.5, 0.5]
                        
                        con = {'type': 'eq', 'fun': constraint}
                        
                        try:
                            result = minimize(objective, 
                                              initial_guess, 
                                              method='SLSQP',
                                              bounds=bounds, 
                                              constraints=con,
                                              options={'maxiter': 100})
                            
                            if result.success:
                                t1_opt, t2_opt = result.x
                                
                                flight_distance = objective(result.x)
                                max_distance = dynamic_battery_model(total_payload)
                                
                                if flight_distance <= max_distance:
                                    takeoff_lat, takeoff_lon = get_point_on_segment(t1_opt, takeoff_start, takeoff_end)
                                    landing_lat, landing_lon = get_point_on_segment(t2_opt, landing_start, landing_end)
                                    
                                    takeoff_virtual = f"V({takeoff_lat:.6f},{takeoff_lon:.6f})"
                                    landing_virtual = f"V({landing_lat:.6f},{landing_lon:.6f})"
                                    
                                    sync_error = abs(constraint(result.x))
                                    score = flight_distance + 2 * sync_error
                                    
                                    if score < best_score:
                                        best_score = score
                                        best_takeoff_segment = takeoff_segment
                                        best_landing_segment = landing_segment
                                        best_params = (takeoff_virtual, landing_virtual)
                        except:
                            continue
                
                if best_params:
                    takeoff_virtual, landing_virtual = best_params
                    optimized_route = [takeoff_virtual] + delivery_nodes + [landing_virtual]
                    optimized_drone_route_list.append(optimized_route)
                else:
                    optimized_drone_route_list.append(route)
            
            truck_drone_routes.append(optimized_drone_route_list)
        
        optimized_drone_routes.append(truck_drone_routes)

    all_drone_routes = validate_drone_virtual_points(optimized_drone_routes, truck_routes, lat_coords, lon_coords)
    return optimized_drone_routes
