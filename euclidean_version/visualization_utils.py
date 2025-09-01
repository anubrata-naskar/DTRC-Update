"""
Visualization utilities for plotting truck routes and drone routes using Euclidean distances
"""
import matplotlib.pyplot as plt
import folium
from folium.features import DivIcon


def plot_truck_routes(truck_routes, lat_coords, lon_coords, save_path=None):
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
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.title("Optimized Truck Routes (Clarke-Wright Savings)")
    plt.grid()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()
    else:
        plt.show()


def plot_combined_routes(truck_routes, all_drone_routes, lat_coords, lon_coords, save_path=None):
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
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.title("Combined Truck and Drone Routes with Virtual Takeoff/Landing Points")
    plt.grid(True)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()
    else:
        plt.show()


def euclidean_distance(lat1, lon1, lat2, lon2):
    """
    Calculate Euclidean distance between two points
    """
    return ((lat2 - lat1) ** 2 + (lon2 - lon1) ** 2) ** 0.5
