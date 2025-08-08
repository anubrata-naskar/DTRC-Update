"""
Drone virtual point validation and adjustment utilities
"""
from distance_utils import haversine_distance, project_point_to_segment


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
