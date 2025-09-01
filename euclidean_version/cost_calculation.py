"""
Cost calculation utilities for drone and truck operations
"""
from distance_utils import euclidean_distance


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
                                    segment_distance = euclidean_distance(prev_lat, prev_lon, next_lat, next_lon)
                                else:
                                    segment_distance = euclidean_distance(prev_lat, prev_lon, lat_coords[next_node], lon_coords[next_node])
                                    
                                flight_distance += segment_distance
                            except ValueError:
                                pass
                    elif isinstance(next_node, str) and next_node.startswith('V('):
                        coords = next_node.strip('V()').split(',')
                        if len(coords) == 2:
                            try:
                                next_lat, next_lon = float(coords[0]), float(coords[1])
                                segment_distance = euclidean_distance(lat_coords[prev_node], lon_coords[prev_node], next_lat, next_lon)
                                flight_distance += segment_distance
                            except ValueError:
                                pass
                    else:
                        segment_distance = euclidean_distance(lat_coords[prev_node], lon_coords[prev_node], lat_coords[next_node], lon_coords[next_node])
                        flight_distance += segment_distance
                    
                    prev_node = next_node
                
                # Calculate drone flight time (distance / drone speed)
                drone_speed = 1.5  # Assuming drone is 1.5x faster than truck
                flight_time = flight_distance / drone_speed
                
                # Add drone operational cost
                drone_operational_cost = fixed_cost + flight_distance * 0.1
                truck_drone_cost += drone_operational_cost
                
                # Calculate truck waiting time at landing point (if any)
                landing_time = None
                takeoff_time = None
                
                # Determine takeoff time
                if isinstance(takeoff_point, str) and takeoff_point.startswith('V('):
                    # For virtual takeoff, estimate time by finding closest truck position
                    takeoff_coords = takeoff_point.strip('V()').split(',')
                    takeoff_lat, takeoff_lon = float(takeoff_coords[0]), float(takeoff_coords[1])
                    
                    min_dist = float('inf')
                    min_time = 0
                    
                    for time, node in truck_schedule:
                        dist = euclidean_distance(takeoff_lat, takeoff_lon, lat_coords[node], lon_coords[node])
                        if dist < min_dist:
                            min_dist = dist
                            min_time = time
                    
                    takeoff_time = min_time
                else:
                    # For node takeoff, find when truck is at that node
                    for time, node in truck_schedule:
                        if node == takeoff_point:
                            takeoff_time = time
                            break
                
                # Determine landing time
                if isinstance(landing_point, str) and landing_point.startswith('V('):
                    landing_coords = landing_point.strip('V()').split(',')
                    landing_lat, landing_lon = float(landing_coords[0]), float(landing_coords[1])
                    
                    min_dist = float('inf')
                    min_time = 0
                    
                    for time, node in truck_schedule:
                        dist = euclidean_distance(landing_lat, landing_lon, lat_coords[node], lon_coords[node])
                        if dist < min_dist:
                            min_dist = dist
                            min_time = time
                    
                    landing_time = min_time
                else:
                    for time, node in truck_schedule:
                        if node == landing_point:
                            landing_time = time
                            break
                
                if takeoff_time is not None and landing_time is not None:
                    drone_arrival_time = takeoff_time + flight_time
                    
                    if drone_arrival_time < landing_time:
                        waiting_time = landing_time - drone_arrival_time
                        truck_waiting_cost += waiting_time * 0.05
        
        print(f"Truck {truck_idx} drone operational cost: {truck_drone_cost}")
        print(f"Truck {truck_idx} waiting cost: {truck_waiting_cost}")
        
        drone_cost += truck_drone_cost
        waiting_cost += truck_waiting_cost
    
    delivery_cost = total_cost + drone_cost + waiting_cost
    
    print(f"Drone operational cost: {drone_cost}")
    print(f"Waiting cost: {waiting_cost}")
    
    return total_cost, delivery_cost
