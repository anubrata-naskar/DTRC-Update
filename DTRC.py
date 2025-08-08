import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler
from scipy.sparse.csgraph import minimum_spanning_tree
import networkx as nx
from networkx.algorithms.approximation import traveling_salesman_problem
from networkx.algorithms.approximation import christofides
import requests
import time
import os
import polyline
import folium

GOOGLE_MAPS_API_KEY = os.getenv('API_KEY')

def dynamic_battery_model(remaining_payload, distance_traveled=0):
    return (155885 / ((200 + remaining_payload) ** 1.5)) - distance_traveled

def read_cvrp_file(file_path):
    with open(file_path, "r") as file:
        lines = file.readlines()

    node_coords = []
    demands = {}
    num_trucks = None
    capacity = None
    section = None

    for line in lines:
        parts = line.strip().split()
        
        if not parts:
            continue

        if parts[0] == "COMMENT" and "No of trucks" in line:
            num_trucks = int(''.join(filter(str.isdigit, line.split("No of trucks:")[-1])))

        elif parts[0] == "CAPACITY":
            capacity = int(parts[-1])

        elif parts[0] == "NODE_COORD_SECTION":
            section = "NODE_COORD"

        elif parts[0] == "DEMAND_SECTION":
            section = "DEMAND"

        elif parts[0] == "DEPOT_SECTION":
            section = "DEPOT"

        elif section == "NODE_COORD":
            node_coords.append((int(parts[0]), float(parts[1]), float(parts[2])))

        elif section == "DEMAND":
            demands[int(parts[0])] = int(parts[1])

    node_coords = np.array(node_coords)
    lat_coords = node_coords[:, 1]
    lon_coords = node_coords[:, 2]

    return num_trucks, capacity, lat_coords, lon_coords, demands

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

def two_opt(route, dist_matrix):
    best = route
    improved = True
    while improved:
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route)):
                if j - i == 1: continue
                new_route = route[:i] + route[i:j][::-1] + route[j:]
                if calculate_route_cost(new_route, dist_matrix) < calculate_route_cost(best, dist_matrix):
                    best = new_route
                    improved = True
        route = best
    return best

def simple_relocate(route, dist_matrix):
    best = route
    for i in range(1, len(route) - 1):
        for j in range(1, len(route)):
            if i == j: continue
            new_route = route[:i] + route[i+1:j] + [route[i]] + route[j:]
            if calculate_route_cost(new_route, dist_matrix) < calculate_route_cost(best, dist_matrix):
                best = new_route
    return best

def swap_move(route, dist_matrix):
    best = route    
    for i in range(1, len(route) - 1):
        for j in range(i + 1, len(route)):
            new_route = route[:i] + [route[j]] + route[i+1:j] + [route[i]] + route[j+1:]
            if calculate_route_cost(new_route, dist_matrix) < calculate_route_cost(best, dist_matrix):
                best = new_route
    return best

def christofides_route(route, dist_matrix):
    G = nx.Graph()
    
    for i in route:
        G.add_node(i)

    for i in route:
        for j in route:
            if i != j:
                G.add_edge(i, j, weight=dist_matrix.iloc[i, j])

    tsp_path = traveling_salesman_problem(G, cycle=True, method=christofides)

    if tsp_path[0] != 0:
        zero_index = tsp_path.index(0)
        tsp_path = tsp_path[zero_index:] + tsp_path[1:zero_index] + [0]

    return tsp_path

def calculate_route_cost(route, dist_matrix):
    cost = 0
    for i in range(len(route) - 1):
        cost += dist_matrix.iloc[route[i], route[i+1]]
    cost += dist_matrix.iloc[route[-1], 0]     
    return cost


def i_k_means(num_trucks, capacity, lat_coords, lon_coords, demands, num_iterations=100):
    coords = np.column_stack((lat_coords[1:], lon_coords[1:]))
    
    demand_weights = np.array([demands.get(i+1, demands.get(i, 0)) for i in range(len(coords))])
    
    scaler = StandardScaler()
    scaled_coords = scaler.fit_transform(coords)
    
    def weighted_kmeans(n_clusters):
        kmeans = KMeans(n_clusters=n_clusters, n_init=10, random_state=42)
        
        kmeans.fit(scaled_coords, sample_weight=demand_weights)
        return kmeans
    
    cluster_model = weighted_kmeans(num_trucks)
    
    cluster_labels = cluster_model.labels_
    
    def construct_cluster_route(cluster_indices):
        if len(cluster_indices) == 0:
            return [0, 0]
        
        cluster_coords_lat = lat_coords[1:][cluster_indices]
        cluster_coords_lon = lon_coords[1:][cluster_indices]
        cluster_node_indices = np.where(np.isin(range(1, len(lat_coords)), cluster_indices + 1))[0] + 1
        
        route = [0]
        current_node = 0
        current_load = 0
        
        unvisited = set(cluster_node_indices)
        
        while unvisited:
            valid_nodes = [
                node for node in unvisited 
                if current_load + demands.get(node, 0) <= capacity
            ]
            
            if not valid_nodes:
                valid_nodes = list(unvisited)
            
            node_scores = []
            for node in valid_nodes:
                node_idx = node - 1
                
                distance = haversine_distance(
                    lat_coords[current_node], lon_coords[current_node],
                    lat_coords[node], lon_coords[node]
                )
                
                demand = demands.get(node, 0)
                importance = demand / (distance + 0.1)
                
                node_scores.append((node, importance))
            
            if node_scores:
                next_node = max(node_scores, key=lambda x: x[1])[0]
                
                route.append(next_node)
                current_load += demands.get(next_node, 0)
                unvisited.remove(next_node)
                current_node = next_node
            else:
                route.append(0)
                break
        
        if route[-1] != 0:
            route.append(0)
        
        return route
    
    initial_routes = []
    for i in range(num_trucks):
        cluster_indices = np.where(cluster_labels == i)[0]
        
        route = construct_cluster_route(cluster_indices)
        initial_routes.append(route)
    
    distances_df = distance_matrix_from_xy(lat_coords, lon_coords, use_google_maps=True)
    
    optimized_routes = []
    for route in initial_routes:
        if len(route) <= 2:
            optimized_routes.append(route)
            continue
            
        possible_routes = {
            "two_opt": two_opt(route, distances_df),
            "simple_relocate": simple_relocate(route, distances_df),
            "swap_move": swap_move(route, distances_df),
            "christofides_route": christofides_route(route, distances_df)
        }
        
        best_function, best_route = min(possible_routes.items(),
            key=lambda item: calculate_route_cost(item[1], distances_df))
        
        print(f"The best route is given by: {best_function}")
        optimized_routes.append(best_route)
    
    return optimized_routes


def apply_dtrc(truck_routes, distances_df, demands, d_capacity, lat_coords, lon_coords):
    import math
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
    import math
    
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

def project_point_to_segment(p_lat, p_lon, s_lat, s_lon, e_lat, e_lon):
    a = e_lat - s_lat
    b = e_lon - s_lon
    
    if a == 0 and b == 0:
        return 0
    
    t = ((p_lat - s_lat) * a + (p_lon - s_lon) * b) / (a * a + b * b)
    return t

def plot_truck_routes(truck_routes, lat_coords, lon_coords):
    plt.figure(figsize=(10, 6))

    for idx, route in enumerate(truck_routes):
        for i in range(len(route) - 1):
            plt.arrow(lon_coords[route[i]], lat_coords[route[i]], 
                      lon_coords[route[i+1]] - lon_coords[route[i]], 
                      lat_coords[route[i+1]] - lat_coords[route[i]], 
                      head_width=0.5, length_includes_head=True, color='blue', alpha=0.8)
        
        plt.scatter([lon_coords[node] for node in route], 
                    [lat_coords[node] for node in route], 
                    color='blue', marker='o', alpha=0.7, label="Truck Route" if idx == 0 else None)

    plt.scatter(lon_coords[0], lat_coords[0], color='black', marker='s', s=100, label="Depot")

    plt.legend()
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Optimized Truck Routes (Clarke-Wright Savings)")
    plt.grid()
    plt.show()

def plot_combined_routes(truck_routes, all_drone_routes, lat_coords, lon_coords):
    plt.figure(figsize=(12, 8))
    
    truck_colors = ['blue', 'green', 'purple', 'brown', 'orange']
    drone_colors = ['red', 'lime', 'magenta', 'chocolate', 'gold']
    
    for idx, route in enumerate(truck_routes):
        truck_color = truck_colors[idx % len(truck_colors)]
        route_nodes = []
        
        for i in range(len(route) - 1):
            plt.arrow(lon_coords[route[i]], lat_coords[route[i]], 
                      lon_coords[route[i+1]] - lon_coords[route[i]], 
                      lat_coords[route[i+1]] - lat_coords[route[i]], 
                      head_width=0.5, length_includes_head=True, color=truck_color, alpha=0.8)
            route_nodes.append(route[i])
        route_nodes.append(route[-1])
        
        plt.scatter([lon_coords[node] for node in route_nodes], 
                    [lat_coords[node] for node in route_nodes], 
                    color=truck_color, marker='o', alpha=0.7, 
                    label=f"Truck {idx+1} Route" if idx == 0 else f"Truck {idx+1}")
        
        for node in route_nodes:
            plt.text(lon_coords[node], lat_coords[node], str(node), fontsize=8, 
                     verticalalignment='bottom', horizontalalignment='right')

    delivery_only_nodes = set()
    takeoff_nodes = set()
    landing_nodes = set()
    virtual_takeoffs = []
    virtual_landings = []
    
    for truck_idx, truck_drone_routes in enumerate(all_drone_routes):
        drone_color = drone_colors[truck_idx % len(drone_colors)]
        
        for drone_idx, drone_route_list in enumerate(truck_drone_routes):
            for route in drone_route_list:
                if not route:
                    continue
                    
                takeoff = route[0]
                landing = route[-1]
                
                delivery_nodes = []
                for node in route[1:-1]:
                    if isinstance(node, (int, float)) and not isinstance(node, str) and not isinstance(node, tuple):
                        delivery_nodes.append(node)
                
                if isinstance(takeoff, str) and takeoff.startswith('V('):
                    coords = takeoff.strip('V()').split(',')
                    if len(coords) == 2:
                        try:
                            lat, lon = float(coords[0]), float(coords[1])
                            virtual_takeoffs.append((lat, lon, truck_idx, drone_idx))
                        except ValueError:
                            pass
                else:
                    takeoff_nodes.add(takeoff)
                
                if isinstance(landing, str) and landing.startswith('V('):
                    coords = landing.strip('V()').split(',')
                    if len(coords) == 2:
                        try:
                            lat, lon = float(coords[0]), float(coords[1])
                            virtual_landings.append((lat, lon, truck_idx, drone_idx))
                        except ValueError:
                            pass
                else:
                    landing_nodes.add(landing)
                
                delivery_only_nodes.update(delivery_nodes)
                
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
                                    plt.arrow(prev_lon, prev_lat,
                                             next_lon - prev_lon, next_lat - prev_lat,
                                             head_width=0.5, length_includes_head=True, color=drone_color,
                                             linestyle='dashed', alpha=0.7)
                                else:
                                    plt.arrow(prev_lon, prev_lat,
                                             lon_coords[next_node] - prev_lon, lat_coords[next_node] - prev_lat,
                                             head_width=0.5, length_includes_head=True, color=drone_color,
                                             linestyle='dashed', alpha=0.7)
                            except ValueError:
                                pass
                    elif isinstance(next_node, str) and next_node.startswith('V('):
                        coords = next_node.strip('V()').split(',')
                        if len(coords) == 2:
                            try:
                                next_lat, next_lon = float(coords[0]), float(coords[1])
                                plt.arrow(lon_coords[prev_node], lat_coords[prev_node],
                                         next_lon - lon_coords[prev_node], next_lat - lat_coords[prev_node],
                                         head_width=0.5, length_includes_head=True, color=drone_color,
                                         linestyle='dashed', alpha=0.7)
                            except ValueError:
                                pass
                    else:
                        plt.arrow(lon_coords[prev_node], lat_coords[prev_node], 
                                 lon_coords[next_node] - lon_coords[prev_node], 
                                 lat_coords[next_node] - lat_coords[prev_node], 
                                 head_width=0.5, length_includes_head=True, color=drone_color, 
                                 linestyle='dashed', alpha=0.7)
                    
                    prev_node = next_node
                
                mid_idx = len(route) // 2
                if mid_idx < len(route) and isinstance(route[mid_idx], (int, float)) and not isinstance(route[mid_idx], str):
                    plt.text(lon_coords[route[mid_idx]], lat_coords[route[mid_idx]], 
                             f"T{truck_idx+1}D{drone_idx+1}", fontsize=8, color=drone_color, weight='bold')
    
    if takeoff_nodes:
        plt.scatter([lon_coords[node] for node in takeoff_nodes], 
                    [lat_coords[node] for node in takeoff_nodes], 
                    color='green', marker='^', s=100, label="Takeoff Points")
    
    if landing_nodes:
        plt.scatter([lon_coords[node] for node in landing_nodes], 
                    [lat_coords[node] for node in landing_nodes], 
                    color='purple', marker='v', s=100, label="Landing Points")
    
    if virtual_takeoffs:
        plt.scatter([lon for lat, lon, _, _ in virtual_takeoffs], 
                    [lat for lat, lon, _, _ in virtual_takeoffs], 
                    color='cyan', marker='*', s=120, label="Moving Truck Takeoff")
        
        for lat, lon, truck_idx, drone_idx in virtual_takeoffs:
            plt.text(lon, lat, f"VT-T{truck_idx+1}D{drone_idx+1}", fontsize=8, 
                     verticalalignment='bottom', horizontalalignment='right', color='cyan')
    
    if virtual_landings:
        plt.scatter([lon for lat, lon, _, _ in virtual_landings], 
                    [lat for lat, lon, _, _ in virtual_landings], 
                    color='magenta', marker='*', s=120, label="Moving Truck Landing")
        
        for lat, lon, truck_idx, drone_idx in virtual_landings:
            plt.text(lon, lat, f"VL-T{truck_idx+1}D{drone_idx+1}", fontsize=8, 
                     verticalalignment='bottom', horizontalalignment='right', color='magenta')
    
    if delivery_only_nodes:
        plt.scatter([lon_coords[node] for node in delivery_only_nodes], 
                    [lat_coords[node] for node in delivery_only_nodes], 
                    color='red', marker='x', s=80, label="Drone Delivery Points")
        
        for node in delivery_only_nodes:
            plt.text(lon_coords[node], lat_coords[node], str(node), fontsize=8, 
                     verticalalignment='top', horizontalalignment='left', color='red')

    plt.scatter(lon_coords[0], lat_coords[0], color='black', marker='s', s=100, label="Depot")
    plt.text(lon_coords[0], lat_coords[0], "0", fontsize=10, color='white',
             verticalalignment='center', horizontalalignment='center')

    plt.legend()
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Combined Truck and Drone Routes with Virtual Takeoff/Landing Points")
    plt.grid(True)
    plt.show()

def plot_google_maps_visualization(truck_routes, all_drone_routes, lat_coords, lon_coords):
    import folium
    from folium.features import DivIcon
    import polyline
    
    truck_colors = ['blue', 'green', 'purple', 'brown', 'orange']
    drone_colors = ['red', 'lime', 'magenta', 'chocolate', 'gold']
    
    center_lat = sum(lat_coords) / len(lat_coords)
    center_lon = sum(lon_coords) / len(lon_coords)
    
    m = folium.Map(location=[center_lat, center_lon], zoom_start=12)
    
    for idx, route in enumerate(truck_routes):
        truck_color = truck_colors[idx % len(truck_colors)]
        
        for i in range(len(route) - 1):
            start = route[i]
            end = route[i+1]
            
            url = f"https://maps.googleapis.com/maps/api/directions/json?origin={lat_coords[start]},{lon_coords[start]}&destination={lat_coords[end]},{lon_coords[end]}&mode=driving&key={GOOGLE_MAPS_API_KEY}"
            response = requests.get(url)
            data = response.json()
            
            if data["status"] == "OK":
                polyline_points = data["routes"][0]["overview_polyline"]["points"]
                decoded_points = polyline.decode(polyline_points)
                
                folium.PolyLine(
                    decoded_points,
                    color=truck_color,
                    weight=4,
                    opacity=0.8,
                    tooltip=f"Truck {idx+1}: {start}-{end}"
                ).add_to(m)
            else:
                folium.PolyLine(
                    [(lat_coords[start], lon_coords[start]), (lat_coords[end], lon_coords[end])],
                    color=truck_color,
                    weight=4,
                    opacity=0.5,
                    tooltip=f"Truck {idx+1}: {start}-{end} (API failed)"
                ).add_to(m)
            
            folium.CircleMarker(
                location=[lat_coords[start], lon_coords[start]],
                radius=5,
                color=truck_color,
                fill=True,
                fill_opacity=0.7,
                tooltip=f"Node {start}"
            ).add_to(m)
            
            folium.CircleMarker(
                location=[lat_coords[end], lon_coords[end]],
                radius=5,
                color=truck_color,
                fill=True,
                fill_opacity=0.7,
                tooltip=f"Node {end}"
            ).add_to(m)
            
            folium.Marker(
                location=[lat_coords[start], lon_coords[start]],
                icon=DivIcon(
                    icon_size=(20,20),
                    icon_anchor=(10,10),
                    html=f'<div style="font-size: 10pt; color: {truck_color};">{start}</div>'
                )
            ).add_to(m)
            
            folium.Marker(
                location=[lat_coords[end], lon_coords[end]],
                icon=DivIcon(
                    icon_size=(20,20),
                    icon_anchor=(10,10),
                    html=f'<div style="font-size: 10pt; color: {truck_color};">{end}</div>'
                )
            ).add_to(m)
    
    for truck_idx, truck_drone_routes in enumerate(all_drone_routes):
        drone_color = drone_colors[truck_idx % len(drone_colors)]
        
        for drone_idx, drone_route_list in enumerate(truck_drone_routes):
            for route in drone_route_list:
                if not route:
                    continue
                
                route_points = []
                
                for node in route:
                    if isinstance(node, (int, float)) and not isinstance(node, str) and not isinstance(node, bool):
                        route_points.append((lat_coords[node], lon_coords[node], node))
                    elif isinstance(node, str) and node.startswith('V('):
                        coords = node.strip('V()').split(',')
                        if len(coords) == 2:
                            try:
                                lat, lon = float(coords[0]), float(coords[1])
                                route_points.append((lat, lon, f"V({lat:.4f},{lon:.4f})"))
                            except ValueError:
                                pass
                
                for i in range(len(route_points) - 1):
                    start_lat, start_lon, start_label = route_points[i]
                    end_lat, end_lon, end_label = route_points[i+1]
                    
                    folium.PolyLine(
                        [(start_lat, start_lon), (end_lat, end_lon)],
                        color=drone_color,
                        weight=2,
                        opacity=0.7,
                        dash_array='5',
                        tooltip=f"Drone {truck_idx+1}-{drone_idx+1}: {start_label}-{end_label}"
                    ).add_to(m)
                    
                    if isinstance(start_label, str) and start_label.startswith('V('):
                        folium.CircleMarker(
                            location=[start_lat, start_lon],
                            radius=3,
                            color='cyan' if i == 0 else 'magenta',
                            fill=True,
                            fill_opacity=0.8,
                            tooltip=f"Virtual {'takeoff' if i == 0 else 'landing'} point: {start_label}"
                        ).add_to(m)
                    
                    if isinstance(end_label, str) and end_label.startswith('V('):
                        folium.CircleMarker(
                            location=[end_lat, end_lon],
                            radius=3,
                            color='magenta',
                            fill=True,
                            fill_opacity=0.8,
                            tooltip=f"Virtual landing point: {end_label}"
                        ).add_to(m)
                    
                    if not (isinstance(start_label, str) and start_label.startswith('V(')):
                        folium.CircleMarker(
                            location=[start_lat, start_lon],
                            radius=4,
                            color='green' if i == 0 else 'red',
                            fill=True,
                            fill_opacity=0.8,
                            tooltip=f"{'Takeoff' if i == 0 else 'Delivery'} node {start_label}"
                        ).add_to(m)
                    
                    if i == len(route_points) - 2 and not (isinstance(end_label, str) and end_label.startswith('V(')):
                        folium.CircleMarker(
                            location=[end_lat, end_lon],
                            radius=4,
                            color='purple',
                            fill=True,
                            fill_opacity=0.8,
                            tooltip=f"Landing node {end_label}"
                        ).add_to(m)

    folium.CircleMarker(
        location=[lat_coords[0], lon_coords[0]],
        radius=8,
        color='black',
        fill=True,
        fill_opacity=0.8,
        tooltip=f"Depot (Node 0)"
    ).add_to(m)
    
    return m

def calculate_total_cost(truck_routes, all_drone_routes, distances_df):
    total_cost = 0
    truck_positions = {}
    
    for route_idx, route in enumerate(truck_routes):
        route_cost = calculate_route_cost(route, distances_df)
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

def optimize_drone_takeoff_landing(truck_routes, drone_routes, distances_df, lat_coords, lon_coords, demands):
    from scipy.optimize import minimize
    import numpy as np
    import math
    
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

file_path = "A-n32-k5.vrp"
d_capacity = 35
num_trucks, capacity, lat_coords, lon_coords, demands = read_cvrp_file(file_path)
num_trucks = 5
print(f"Number of trucks: {num_trucks}")
print(demands)
truck_routes = i_k_means(num_trucks, capacity, lat_coords, lon_coords, demands)

print("-----Truck routes-----")
for route in truck_routes:
    print([int(x) for x in route])
plot_truck_routes(truck_routes, lat_coords, lon_coords)

distances_df = distance_matrix_from_xy(lat_coords, lon_coords, use_google_maps=True)
drone_routes, modified_truck_routes = apply_dtrc(truck_routes, distances_df, demands, d_capacity, lat_coords, lon_coords)

print("Optimizing drone takeoff and landing points...")
drone_routes = optimize_drone_takeoff_landing(modified_truck_routes, drone_routes, 
                                                       distances_df, lat_coords, lon_coords, demands)
print("-----Modified Truck routes-----")
for route in modified_truck_routes:
    print([int(x) for x in route])

formatted_drone_routes = []
for truck_drones in drone_routes:
    truck_formatted = []
    for drone_trips in truck_drones:
        drone_formatted = []
        for route in drone_trips:
            route_formatted = []
            for node in route:
                if isinstance(node, (int, float)) and not isinstance(node, str) and not isinstance(node, bool):
                    route_formatted.append(int(node))
                else:
                    route_formatted.append(node)
            drone_formatted.append(route_formatted)
        truck_formatted.append(drone_formatted)
    formatted_drone_routes.append(truck_formatted)

print("-----Drone routes-----")
print(formatted_drone_routes)

plot_combined_routes(modified_truck_routes, drone_routes, lat_coords, lon_coords)

google_map = plot_google_maps_visualization(modified_truck_routes, drone_routes, lat_coords, lon_coords)
google_map.save("truck_drone_routes.html")
print("Interactive map saved as truck_drone_routes.html")

total_cost, delivery_cost = calculate_total_cost(modified_truck_routes, drone_routes, distances_df)
print(f"Total Cost: {total_cost}")
print(f"Delivery Cost: {delivery_cost}")