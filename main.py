"""
Main execution script for the DTRC (Drone-Truck Routing Coordination) system
"""
from file_utils import read_cvrp_file
from distance_utils import distance_matrix_from_xy
from clustering_utils import i_k_means
from dtrc_algorithm import apply_dtrc
from drone_optimization import optimize_drone_takeoff_landing
from advanced_drone_optimization import optimize_drone_takeoff_landing_advanced
from cost_calculation import calculate_total_cost
from visualization_utils import plot_truck_routes, plot_combined_routes, plot_google_maps_visualization


def main():
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

    print("Optimizing drone takeoff and landing points with advanced route analysis...")
    
    # Choice between basic and advanced optimization
    use_advanced_optimization = True  # Set to False for basic optimization
    
    if use_advanced_optimization:
        drone_routes = optimize_drone_takeoff_landing_advanced(
            modified_truck_routes, drone_routes, distances_df, 
            lat_coords, lon_coords, demands,
            truck_speed=1.0, drone_speed=1.5
        )
    else:
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

    total_cost, delivery_cost = calculate_total_cost(modified_truck_routes, drone_routes, distances_df, lat_coords, lon_coords)
    print(f"Total Cost: {total_cost}")
    print(f"Delivery Cost: {delivery_cost}")


if __name__ == "__main__":
    main()
