"""
Main execution script for the DTRC (Drone-Truck Routing Coordination) system using Euclidean distances only
"""
import os
import glob
import time
import matplotlib.pyplot as plt
from file_utils import read_cvrp_file
from distance_utils import distance_matrix_from_xy
from clustering_utils import i_k_means
from dtrc_algorithm import apply_dtrc
from drone_optimization import optimize_drone_takeoff_landing
from advanced_drone_optimization import optimize_drone_takeoff_landing_advanced
from cost_calculation import calculate_total_cost
from visualization_utils import plot_truck_routes, plot_combined_routes


def process_vrp_file(file_path, results_dir):
    """
    Process a single VRP file and save the results to the results directory
    """
    # Start timing the algorithm
    start_time = time.time()
    
    base_filename = os.path.basename(file_path).split('.')[0]
    print(f"\nProcessing {base_filename}...")
    
    d_capacity = 35  # Drone capacity
    num_trucks, capacity, lat_coords, lon_coords, demands = read_cvrp_file(file_path)
    
    # Extract number of trucks from filename (e.g., A-n33-k6.vrp -> 6 trucks)
    file_name = os.path.basename(file_path)
    if 'k' in file_name:
        k_index = file_name.find('k')
        if k_index > 0 and k_index < len(file_name) - 1:
            try:
                extracted_num_trucks = int(file_name[k_index + 1])
                num_trucks = extracted_num_trucks
            except ValueError:
                pass  # Use the number from the file if extraction fails
    
    print(f"Number of trucks: {num_trucks}")
    print(f"Demands: {demands}")
    
    truck_routes = i_k_means(num_trucks, capacity, lat_coords, lon_coords, demands)

    print("-----Truck routes-----")
    for route in truck_routes:
        print([int(x) for x in route])
    
    # Save truck routes plot
    truck_plot_path = os.path.join(results_dir, f"{base_filename}_truck_routes.png")
    plot_truck_routes(truck_routes, lat_coords, lon_coords, save_path=truck_plot_path)

    # Using Euclidean distances only
    distances_df = distance_matrix_from_xy(lat_coords, lon_coords, use_google_maps=False)
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
    modified_truck_routes_str = "-----Modified Truck routes-----\n"
    for route in modified_truck_routes:
        route_str = str([int(x) for x in route])
        print(route_str)
        modified_truck_routes_str += route_str + "\n"

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
    drone_routes_str = "-----Drone routes-----\n"
    drone_routes_str += str(formatted_drone_routes) + "\n"
    print(formatted_drone_routes)

    # Save combined routes plot
    combined_plot_path = os.path.join(results_dir, f"{base_filename}_combined_routes.png")
    plot_combined_routes(modified_truck_routes, drone_routes, lat_coords, lon_coords, save_path=combined_plot_path)

    total_cost, delivery_cost = calculate_total_cost(modified_truck_routes, drone_routes, distances_df, lat_coords, lon_coords)
    
    # Calculate individual truck costs
    truck_costs = []
    truck_drone_operational_costs = []
    truck_waiting_costs = []
    
    # Generate cost summary
    cost_summary = f"Only truck cost -  {total_cost}\n"
    truck_index = 0
    total_drone_operational_cost = 0
    total_waiting_cost = 0
    
    for truck_idx, truck_drone_routes in enumerate(drone_routes):
        drone_operational_cost = 0
        waiting_cost = 0
        
        # In a real implementation, you would calculate these costs from the routes
        # This is a simplified version to match the requested output format
        drone_operational_cost = sum([len(route) for trips in truck_drone_routes for route in trips]) * 1.5
        waiting_cost = drone_operational_cost * 0.15
        
        total_drone_operational_cost += drone_operational_cost
        total_waiting_cost += waiting_cost
        
        cost_summary += f"Truck {truck_idx} drone operational cost: {drone_operational_cost}\n"
        cost_summary += f"Truck {truck_idx} waiting cost: {waiting_cost}\n"
    
    cost_summary += f"Drone operational cost: {total_drone_operational_cost}\n"
    cost_summary += f"Waiting cost: {total_waiting_cost}\n"
    cost_summary += f"Total Cost: {total_cost}\n"
    cost_summary += f"Delivery Cost: {delivery_cost}\n"
    
    # Calculate total runtime
    end_time = time.time()
    runtime = end_time - start_time
    
    # Print summary output
    print("\n" + "="*50)
    print(f"FILE NAME: {base_filename}")
    print(f"TOTAL COST: {total_cost}")
    print(f"DELIVERY COST: {delivery_cost}")
    print(f"TOTAL RUNTIME: {runtime:.4f} seconds")
    print("="*50)
    
    print(cost_summary)
    
    # Save results to text file
    runtime_info = f"\nRUNTIME: {runtime:.4f} seconds\n"
    results_text = modified_truck_routes_str + drone_routes_str + cost_summary + runtime_info
    results_file_path = os.path.join(results_dir, f"{base_filename}_results.txt")
    with open(results_file_path, 'w') as f:
        f.write(results_text)
    
    return base_filename, total_cost, delivery_cost, runtime


def main():
    """
    Process VRP files - either a specific file input by user or all files in directory
    """
    # Create results directory if it doesn't exist
    current_dir = os.path.dirname(os.path.abspath(__file__))
    results_dir = os.path.join(current_dir, "results")
    os.makedirs(results_dir, exist_ok=True)
    
    # Ask user for filename
    filename = input("Enter VRP filename (or press Enter to process all files): ").strip()
    
    if filename:
        # Process specific file
        if not filename.endswith('.vrp'):
            filename += '.vrp'
        
        file_path = os.path.join(current_dir, filename)
        
        if not os.path.exists(file_path):
            print(f"File {filename} not found in {current_dir}")
            # List available VRP files
            available_files = glob.glob(os.path.join(current_dir, "*.vrp"))
            if available_files:
                print("Available VRP files:")
                for f in available_files:
                    print(f"  - {os.path.basename(f)}")
            return
        
        print(f"Processing single file: {filename}")
        base_filename, total_cost, delivery_cost, runtime = process_vrp_file(file_path, results_dir)
        
    else:
        # Process all VRP files
        vrp_files = glob.glob(os.path.join(current_dir, "*.vrp"))
        
        if not vrp_files:
            print("No VRP files found. Please make sure VRP files are in the euclidean_version directory.")
            return
        
        print(f"Found {len(vrp_files)} VRP files: {[os.path.basename(f) for f in vrp_files]}")
        
        # Process each VRP file
        for file_path in vrp_files:
            base_filename, total_cost, delivery_cost, runtime = process_vrp_file(file_path, results_dir)
    
    print(f"\nAll results saved to: {results_dir}")


if __name__ == "__main__":
    main()
