"""
Cost calculation utilities for drone and truck operations
"""
from distance_utils import haversine_distance


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
