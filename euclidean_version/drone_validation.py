"""
Drone virtual point validation and adjustment utilities
"""
from distance_utils import euclidean_distance, project_point_to_segment


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
                lat_coords[start_node], lon_coords[start_node],
                lat_coords[end_node], lon_coords[end_node]
            ))
        
        for drone_idx, drone_routes in enumerate(truck_drone_routes):
            for route_idx, route in enumerate(drone_routes):
                if not route:
                    continue
                
                modified_route = []
                
                for node_idx, node in enumerate(route):
                    if isinstance(node, str) and node.startswith('V('):
                        coords = node.strip('V()').split(',')
                        virtual_lat, virtual_lon = float(coords[0]), float(coords[1])
                        
                        best_segment = None
                        best_dist = float('inf')
                        best_projection = None
                        
                        for segment_idx, segment in enumerate(truck_segments):
                            start_node, end_node, s_lat, s_lon, e_lat, e_lon = segment
                            
                            t = project_point_to_segment(virtual_lat, virtual_lon, s_lat, s_lon, e_lat, e_lon)
                            t = max(0, min(1, t))
                            
                            proj_lat = s_lat + t * (e_lat - s_lat)
                            proj_lon = s_lon + t * (e_lon - s_lon)
                            
                            dist = euclidean_distance(virtual_lat, virtual_lon, proj_lat, proj_lon)
                            
                            if dist < best_dist:
                                best_dist = dist
                                best_segment = segment
                                best_projection = (t, proj_lat, proj_lon)
                        
                        if best_segment:
                            start_node, end_node, s_lat, s_lon, e_lat, e_lon = best_segment
                            t, proj_lat, proj_lon = best_projection
                            
                            adjusted_virtual = f"V({proj_lat:.6f},{proj_lon:.6f})"
                            modified_route.append(adjusted_virtual)
                        else:
                            modified_route.append(node)
                    else:
                        modified_route.append(node)
                
                all_drone_routes[truck_idx][drone_idx][route_idx] = modified_route
    
    return all_drone_routes


def project_point_to_segment(p_lat, p_lon, s_lat, s_lon, e_lat, e_lon):
    a = e_lat - s_lat
    b = e_lon - s_lon
    
    if a == 0 and b == 0:
        return 0
    
    t = ((p_lat - s_lat) * a + (p_lon - s_lon) * b) / (a * a + b * b)
    return t
