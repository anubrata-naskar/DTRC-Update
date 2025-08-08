"""
K-means clustering utilities for vehicle routing
"""
import numpy as np
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler
from distance_utils import distance_matrix_from_xy, haversine_distance
from optimization_utils import two_opt, simple_relocate, swap_move, christofides_route, calculate_route_cost


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
