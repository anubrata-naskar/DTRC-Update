"""
Drone and DTRC (Drone-Truck Routing Coordination) utilities
"""
from itertools import combinations
import numpy as np
from scipy.optimize import minimize
from distance_utils import haversine_distance, project_point_to_segment
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


def validate_drone_virtual_points(all_drone_routes, modified_truck_routes, lat_coords, lon_coords):
    
    for truck_idx, truck_drone_routes in enumerate(all_drone_routes):
        if truck_idx >= len(modified_truck_routes) or not modified_truck_routes[truck_idx]:
            continue
            
        truck_route = modified_truck_routes[truck_idx]
        
        truck_segments = []
        for i in range(len(truck_route) - 1):
            start_node = truck_route[i]
            end_node = truck_route[i+1]
            truck_segments.append((
                start_node, end_node,
                (lat_coords[start_node], lon_coords[start_node]),
                (lat_coords[end_node], lon_coords[end_node])
            ))
        
        for drone_idx, drone_routes in enumerate(truck_drone_routes):
            for route_idx, route in enumerate(drone_routes):
                if not route:
                    continue
                
                takeoff = route[0]
                if isinstance(takeoff, str) and takeoff.startswith('V('):
                    coords = takeoff.strip('V()').split(',')
                    if len(coords) == 2:
                        try:
                            v_lat, v_lon = float(coords[0]), float(coords[1])
                            
                            on_route = False
                            best_segment = None
                            best_distance = float('inf')
                            
                            for seg_idx, (start_node, end_node, start_coords, end_coords) in enumerate(truck_segments):
                                s_lat, s_lon = start_coords
                                e_lat, e_lon = end_coords
                                
                                seg_len_sq = haversine_distance(s_lat, s_lon, e_lat, e_lon)**2
                                
                                if seg_len_sq < 0.0001:
                                    dist = haversine_distance(v_lat, v_lon, s_lat, s_lon)
                                else:
                                    t = max(0, min(1, project_point_to_segment(v_lat, v_lon, s_lat, s_lon, e_lat, e_lon)))
                                    
                                    proj_lat = s_lat + t * (e_lat - s_lat)
                                    proj_lon = s_lon + t * (e_lon - s_lon)
                                    
                                    dist = haversine_distance(v_lat, v_lon, proj_lat, proj_lon)
                                
                                if dist < 0.1:
                                    on_route = True
                                    break
                                    
                                if dist < best_distance:
                                    best_distance = dist
                                    best_segment = (seg_idx, start_node, end_node, t)
                            
                            if not on_route and best_segment:
                                seg_idx, start_node, end_node, t = best_segment
                                
                                s_lat, s_lon = lat_coords[start_node], lon_coords[start_node]
                                e_lat, e_lon = lat_coords[end_node], lon_coords[end_node]
                                
                                new_lat = s_lat + t * (e_lat - s_lat)
                                new_lon = s_lon + t * (e_lon - s_lon)
                                
                                new_takeoff = f"V({new_lat:.6f},{new_lon:.6f})"
                                
                                all_drone_routes[truck_idx][drone_idx][route_idx][0] = new_takeoff
                                
                                print(f"Shifted virtual takeoff from {takeoff} to {new_takeoff} for Truck {truck_idx+1}, Drone {drone_idx+1}")
                                
                        except ValueError:
                            pass
                
                landing = route[-1]
                if isinstance(landing, str) and landing.startswith('V('):
                    coords = landing.strip('V()').split(',')
                    if len(coords) == 2:
                        try:
                            v_lat, v_lon = float(coords[0]), float(coords[1])
                            
                            on_route = False
                            best_segment = None
                            best_distance = float('inf')
                            
                            for seg_idx, (start_node, end_node, start_coords, end_coords) in enumerate(truck_segments):
                                s_lat, s_lon = start_coords
                                e_lat, e_lon = end_coords
                                
                                seg_len_sq = haversine_distance(s_lat, s_lon, e_lat, e_lon)**2
                                
                                if seg_len_sq < 0.0001:
                                    dist = haversine_distance(v_lat, v_lon, s_lat, s_lon)
                                else:
                                    t = max(0, min(1, project_point_to_segment(v_lat, v_lon, s_lat, s_lon, e_lat, e_lon)))
                                    
                                    proj_lat = s_lat + t * (e_lat - s_lat)
                                    proj_lon = s_lon + t * (e_lon - s_lon)
                                    
                                    dist = haversine_distance(v_lat, v_lon, proj_lat, proj_lon)
                                
                                if dist < 0.1:
                                    on_route = True
                                    break
                                    
                                if dist < best_distance:
                                    best_distance = dist
                                    best_segment = (seg_idx, start_node, end_node, t)
                            
                            if not on_route and best_segment:
                                seg_idx, start_node, end_node, t = best_segment
                                
                                s_lat, s_lon = lat_coords[start_node], lon_coords[start_node]
                                e_lat, e_lon = lat_coords[end_node], lon_coords[end_node]
                                
                                new_lat = s_lat + t * (e_lat - s_lat)
                                new_lon = s_lon + t * (e_lon - s_lon)
                                
                                new_landing = f"V({new_lat:.6f},{new_lon:.6f})"
                                
                                all_drone_routes[truck_idx][drone_idx][route_idx][-1] = new_landing
                                
                                print(f"Shifted virtual landing from {landing} to {new_landing} for Truck {truck_idx+1}, Drone {drone_idx+1}")
                                
                        except ValueError:
                            pass
                            
    return all_drone_routes


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


def calculate_total_cost(truck_routes, all_drone_routes, distances_df, lat_coords, lon_coords):
    total_cost = 0
    truck_positions = {}
    
    for route_idx, route in enumerate(truck_routes):
        route_cost = 0
        for i in range(len(route) - 1):
            route_cost += distances_df.iloc[route[i], route[i+1]]
        route_cost += distances_df.iloc[route[-1], 0]
        total_cost += route_cost
        
        current_time = 0
        truck_positions[route_idx] = [(0, route[0])]
        
        for i in range(len(route) - 1):
            segment_time = distances_df.iloc[route[i], route[i+1]]
            current_time += segment_time
            truck_positions[route_idx].append((current_time, route[i+1]))
    
    print("Only truck cost - ", total_cost)    
    
    drone_cost = 0
    waiting_cost = 0
    
    for truck_idx, truck_drone_routes in enumerate(all_drone_routes):
        truck_drone_cost = 0
        truck_waiting_cost = 0
        
        truck_schedule = truck_positions[truck_idx]
        
        for drone_idx, drone_route_list in enumerate(truck_drone_routes):
            for route in drone_route_list:
                if not route:
                    continue
                    
                takeoff_point = route[0]
                landing_point = route[-1]
                
                fixed_cost = 2
                flight_distance = 0
                
                prev_node = route[0]
                for next_node in route[1:]:
                    if isinstance(prev_node, str) and prev_node.startswith('V('):
                        coords = prev_node.strip('V()').split(',')
                        if len(coords) == 2:
                            try:
                                prev_lat, prev_lon = float(coords[0]), float(coords[1])
                                
                                if isinstance(next_node, str) and next_node.startswith('V('):
                                    coords = next_node.strip('V()').split(',')
                                    next_lat, next_lon = float(coords[0]), float(coords[1])
                                    segment_distance = haversine_distance(prev_lat, prev_lon, next_lat, next_lon)
                                else:
                                    segment_distance = haversine_distance(prev_lat, prev_lon, lat_coords[next_node], lon_coords[next_node])
                                
                                flight_distance += segment_distance
                            except ValueError:
                                pass
                    elif isinstance(next_node, str) and next_node.startswith('V('):
                        coords = next_node.strip('V()').split(',')
                        if len(coords) == 2:
                            try:
                                next_lat, next_lon = float(coords[0]), float(coords[1])
                                segment_distance = haversine_distance(lat_coords[prev_node], lon_coords[prev_node], next_lat, next_lon)
                                flight_distance += segment_distance
                            except ValueError:
                                pass
                    else:
                        segment_distance = haversine_distance(lat_coords[prev_node], lon_coords[prev_node], 
                                                             lat_coords[next_node], lon_coords[next_node])
                        flight_distance += segment_distance
                    
                    prev_node = next_node
                
                flight_time = flight_distance / 1.5
                
                route_cost = flight_time + fixed_cost
                truck_drone_cost += route_cost
        
        print(f"Truck {truck_idx+1} drone cost - {truck_drone_cost}")
        print(f"Truck {truck_idx+1} waiting cost - {truck_waiting_cost}")
        
        drone_cost += truck_drone_cost
        waiting_cost += truck_waiting_cost
    
    print("Total drone cost - ", drone_cost)
    print("Total waiting cost - ", waiting_cost)
    
    total_cost += drone_cost + waiting_cost
    
    delivery_cost = 0
    for route in truck_routes:
        if len(route) > 2:
            for i in range(1, len(route) - 2):
                delivery_cost += distances_df.iloc[route[i], route[i+1]]
    
    for truck_drone_routes in all_drone_routes:
        for drone_route_list in truck_drone_routes:
            for route in drone_route_list:
                if not route:
                    continue
                    
                delivery_cost += 2
                
                prev_node = route[0]
                for next_node in route[1:]:
                    if isinstance(prev_node, str) and prev_node.startswith('V('):
                        coords = prev_node.strip('V()').split(',')
                        if len(coords) == 2:
                            try:
                                prev_lat, prev_lon = float(coords[0]), float(coords[1])
                                
                                if isinstance(next_node, str) and next_node.startswith('V('):
                                    coords = next_node.strip('V()').split(',')
                                    next_lat, next_lon = float(coords[0]), float(coords[1])
                                    segment_distance = haversine_distance(prev_lat, prev_lon, next_lat, next_lon)
                                else:
                                    segment_distance = haversine_distance(prev_lat, prev_lon, lat_coords[next_node], lon_coords[next_node])
                                
                                delivery_cost += segment_distance / 1.5
                            except ValueError:
                                pass
                    elif isinstance(next_node, str) and next_node.startswith('V('):
                        coords = next_node.strip('V()').split(',')
                        if len(coords) == 2:
                            try:
                                next_lat, next_lon = float(coords[0]), float(coords[1])
                                segment_distance = haversine_distance(lat_coords[prev_node], lon_coords[prev_node], next_lat, next_lon)
                                delivery_cost += segment_distance / 1.5
                            except ValueError:
                                pass
                    else:
                        segment_distance = haversine_distance(lat_coords[prev_node], lon_coords[prev_node], 
                                                             lat_coords[next_node], lon_coords[next_node])
                        delivery_cost += segment_distance / 1.5
                    
                    prev_node = next_node
    
    return total_cost, delivery_cost
