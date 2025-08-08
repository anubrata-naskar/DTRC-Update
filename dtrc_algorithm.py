"""
Core DTRC (Drone-Truck Routing Coordination) algorithm
"""
from itertools import combinations
import numpy as np
from distance_utils import haversine_distance
from file_utils import dynamic_battery_model


def apply_dtrc(truck_routes, distances_df, demands, d_capacity, lat_coords, lon_coords):
    from itertools import combinations
    
    DRONE_CAPACITY = d_capacity
    
    all_drone_routes = []
    modified_truck_routes = [route.copy() for route in truck_routes]
    
    for route_idx, truck_route in enumerate(truck_routes):
        truck_drone_routes = [[] for _ in range(2)]
        
        used_nodes = set()
        
        drone_positions = [0, 0]
        
        i = 0
        while i < len(truck_route) - 1:
            current_node = truck_route[i]
            next_node = truck_route[i+1]
            best_route = None
            best_landing_idx = None
            best_drone = None
            
            takeoff_options = [
                (current_node, i, False),
                (f"V({(lat_coords[current_node] + lat_coords[next_node])/2:.6f},{(lon_coords[current_node] + lon_coords[next_node])/2:.6f})", i + 0.5, True)
            ]
            
            for takeoff_point, takeoff_idx, is_virtual_takeoff in takeoff_options:
                for drone_idx in range(2):
                    if drone_positions[drone_idx] > takeoff_idx:
                        continue
                    
                    potential_deliveries = []
                    
                    max_search = min(i + 8, len(truck_route) - 1)
                    
                    for j in range(i + 1, max_search):
                        candidate_node = truck_route[j]
                        
                        if candidate_node in used_nodes:
                            continue
                        
                        weight = demands.get(candidate_node, demands.get(candidate_node+1, 0))
                        
                        if weight <= DRONE_CAPACITY:
                            if is_virtual_takeoff:
                                coords = takeoff_point.strip('V()').split(',')
                                from_lat, from_lon = float(coords[0]), float(coords[1])
                                
                                D = haversine_distance(from_lat, from_lon, lat_coords[candidate_node], lon_coords[candidate_node])
                            else:
                                D = distances_df.iloc[takeoff_point, candidate_node]
                                
                            potential_deliveries.append((candidate_node, j, D, weight, D / max(1, weight)))
                    
                    potential_deliveries.sort(key=lambda x: x[1])
                    
                    if potential_deliveries:
                        best_value_for_this_drone = 0
                        
                        max_nodes = min(2, len(potential_deliveries))
                        
                        for num_nodes in range(1, max_nodes + 1):
                            max_candidates = min(4, len(potential_deliveries))
                            
                            for combo_indices in combinations(range(max_candidates), min(num_nodes, max_candidates)):
                                combo = [potential_deliveries[idx] for idx in sorted(combo_indices)]
                                
                                delivery_nodes = []
                                indices = []
                                total_payload = 0
                                
                                for node, idx, _, weight, _ in combo:
                                    delivery_nodes.append(node)
                                    indices.append(idx)
                                    total_payload += weight
                                
                                if total_payload > DRONE_CAPACITY:
                                    continue
                                
                                last_delivery_idx = max(indices)
                                
                                landing_options = []
                                
                                if last_delivery_idx + 1 < len(truck_route):
                                    landing_node = truck_route[last_delivery_idx + 1]
                                    landing_options.append((landing_node, last_delivery_idx + 1, False))
                                
                                if last_delivery_idx + 2 < len(truck_route):
                                    node1 = truck_route[last_delivery_idx + 1]
                                    node2 = truck_route[last_delivery_idx + 2]
                                    
                                    virtual_lat = (lat_coords[node1] + lat_coords[node2]) / 2
                                    virtual_lon = (lon_coords[node1] + lon_coords[node2]) / 2
                                    virtual_landing = f"V({virtual_lat:.6f},{virtual_lon:.6f})"
                                    
                                    landing_options.append((virtual_landing, last_delivery_idx + 1.5, True))
                                
                                for landing_point, landing_idx, is_virtual_landing in landing_options:
                                    route = [takeoff_point]
                                    route.extend(delivery_nodes)
                                    route.append(landing_point)
                                    
                                    valid_route = True
                                    remaining_payload = total_payload
                                    battery_consumed = 0
                                    total_dist = 0
                                    
                                    for idx in range(len(route) - 1):
                                        from_node = route[idx]
                                        to_node = route[idx + 1]
                                        
                                        if isinstance(from_node, str) and from_node.startswith('V('):
                                            coords = from_node.strip('V()').split(',')
                                            from_lat, from_lon = float(coords[0]), float(coords[1])
                                            
                                            if isinstance(to_node, str) and to_node.startswith('V('):
                                                coords = to_node.strip('V()').split(',')
                                                to_lat, to_lon = float(coords[0]), float(coords[1])
                                                segment_distance = haversine_distance(from_lat, from_lon, to_lat, to_lon)
                                            else:
                                                segment_distance = haversine_distance(from_lat, from_lon, lat_coords[to_node], lon_coords[to_node])
                                        elif isinstance(to_node, str) and to_node.startswith('V('):
                                            coords = to_node.strip('V()').split(',')
                                            to_lat, to_lon = float(coords[0]), float(coords[1])
                                            segment_distance = haversine_distance(lat_coords[from_node], lon_coords[from_node], to_lat, to_lon)
                                        else:
                                            segment_distance = haversine_distance(lat_coords[from_node], lon_coords[from_node], 
                                                                                 lat_coords[to_node], lon_coords[to_node])
                                        
                                        max_distance = dynamic_battery_model(remaining_payload, battery_consumed)
                                        if segment_distance > max_distance:
                                            valid_route = False
                                            break
                                        
                                        battery_consumed += segment_distance
                                        total_dist += segment_distance
                                        
                                        if to_node in delivery_nodes:
                                            node_weight = demands.get(to_node, demands.get(to_node+1, 0))
                                            remaining_payload -= node_weight
                                    
                                    if valid_route:
                                        value = len(delivery_nodes) / (total_dist + 0.1)
                                        
                                        if value > best_value_for_this_drone:
                                            best_value_for_this_drone = value
                                            best_route = route
                                            best_landing_idx = landing_idx
                                            best_drone = drone_idx
                        
                        if best_route:
                            break
                
                if best_route:
                    break
            
            if best_route:
                truck_drone_routes[best_drone].append(best_route)
                
                for node in best_route[1:-1]:
                    if not isinstance(node, str):
                        used_nodes.add(node)
                
                drone_positions[best_drone] = best_landing_idx
                
                i = int(best_landing_idx)
            else:
                i += 1
        
        all_drone_routes.append(truck_drone_routes)
    
    # Import here to avoid circular imports
    from drone_validation import validate_drone_virtual_points
    
    for route_idx, route in enumerate(modified_truck_routes):
        nodes_to_remove = set()
        
        for drone_idx, drone_routes in enumerate(all_drone_routes[route_idx]):
            for drone_route in drone_routes:
                for node in drone_route[1:-1]:
                    if not isinstance(node, str):
                        nodes_to_remove.add(int(node))
        
        modified_truck_routes[route_idx] = [node for node in route if int(node) not in nodes_to_remove]
        all_drone_routes = validate_drone_virtual_points(all_drone_routes, modified_truck_routes, lat_coords, lon_coords)
    
    return all_drone_routes, modified_truck_routes
