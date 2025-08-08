"""
Advanced drone takeoff/landing optimization using Optuna and detailed route analysis
"""
import optuna
import numpy as np
from route_analysis import get_route_data
from distance_utils import haversine_distance


class DroneOptimizationProblem:
    """Represents a drone optimization problem with timing constraints"""
    
    def __init__(self, truck_route_segments, customer_lat, customer_lon, 
                 truck_speed=1.0, drone_speed=1.5):
        self.truck_route_segments = truck_route_segments  # List of RouteData objects
        self.customer_lat = customer_lat
        self.customer_lon = customer_lon
        self.truck_speed = truck_speed
        self.drone_speed = drone_speed
        
        # Calculate cumulative distances for segment selection
        self.segment_boundaries = []
        cumulative_distance = 0
        
        for route_data in truck_route_segments:
            self.segment_boundaries.append(cumulative_distance)
            cumulative_distance += route_data.total_distance
        
        self.total_route_distance = cumulative_distance
        self.segment_boundaries.append(cumulative_distance)
    
    def get_position_on_route(self, distance_along_route):
        """Get lat/lon position at given distance along complete truck route"""
        if distance_along_route <= 0:
            first_route = self.truck_route_segments[0]
            return (first_route.start_lat, first_route.start_lon)
        
        if distance_along_route >= self.total_route_distance:
            last_route = self.truck_route_segments[-1]
            return (last_route.end_lat, last_route.end_lon)
        
        # Find which segment contains this distance
        for i, route_data in enumerate(self.truck_route_segments):
            segment_start = self.segment_boundaries[i]
            segment_end = self.segment_boundaries[i + 1]
            
            if segment_start <= distance_along_route <= segment_end:
                # Distance within this route segment
                distance_in_segment = distance_along_route - segment_start
                time_in_segment = distance_in_segment / self.truck_speed
                
                return route_data.get_truck_position_at_time(time_in_segment, self.truck_speed)
        
        return None
    
    def calculate_drone_mission_time(self, takeoff_distance, landing_distance):
        """Calculate time for drone mission given takeoff and landing distances"""
        if takeoff_distance >= landing_distance:
            return float('inf')  # Invalid: landing before takeoff
        
        # Get takeoff and landing positions
        takeoff_pos = self.get_position_on_route(takeoff_distance)
        landing_pos = self.get_position_on_route(landing_distance)
        
        if not takeoff_pos or not landing_pos:
            return float('inf')
        
        takeoff_lat, takeoff_lon = takeoff_pos
        landing_lat, landing_lon = landing_pos
        
        # Calculate drone flight distances
        dist_to_customer = haversine_distance(takeoff_lat, takeoff_lon, 
                                            self.customer_lat, self.customer_lon)
        dist_from_customer = haversine_distance(self.customer_lat, self.customer_lon,
                                              landing_lat, landing_lon)
        
        # Calculate drone flight time
        drone_flight_time = (dist_to_customer + dist_from_customer) / self.drone_speed
        
        # Calculate truck travel time between takeoff and landing
        truck_travel_distance = landing_distance - takeoff_distance
        truck_travel_time = truck_travel_distance / self.truck_speed
        
        return drone_flight_time, truck_travel_time
    
    def objective(self, trial):
        """Optuna objective function to minimize total time"""
        # Suggest takeoff and landing positions along route
        takeoff_distance = trial.suggest_float('takeoff_distance', 0, 
                                             self.total_route_distance * 0.8)
        landing_distance = trial.suggest_float('landing_distance', 
                                             takeoff_distance + 0.1,  # Minimum gap
                                             self.total_route_distance)
        
        drone_time, truck_time = self.calculate_drone_mission_time(takeoff_distance, landing_distance)
        
        if drone_time == float('inf'):
            return float('inf')
        
        # Objective: minimize the maximum of drone time and truck time
        # (both must be completed for mission to finish)
        total_mission_time = max(drone_time, truck_time)
        
        # Add penalty for timing mismatch (encourages synchronization)
        timing_penalty = abs(drone_time - truck_time) * 0.1
        
        return total_mission_time + timing_penalty


def optimize_drone_takeoff_landing_advanced(truck_routes, drone_routes, distances_df, 
                                          lat_coords, lon_coords, demands, 
                                          truck_speed=1.0, drone_speed=1.5):
    """
    Advanced optimization of drone takeoff/landing points using Optuna and detailed route analysis
    
    Args:
        truck_routes: List of truck routes
        drone_routes: List of drone routes for each truck
        distances_df: Distance matrix
        lat_coords, lon_coords: Coordinate arrays
        demands: Demand data
        truck_speed: Truck speed (default 1.0)
        drone_speed: Drone speed multiplier (default 1.5)
    
    Returns:
        Optimized drone routes
    """
    print("=== Advanced Drone Optimization with Optuna ===")
    optimized_drone_routes = []
    
    for truck_idx, (truck_route, truck_drone_routes) in enumerate(zip(truck_routes, drone_routes)):
        print(f"\nOptimizing Truck {truck_idx + 1} routes...")
        
        if not truck_route or len(truck_route) < 2:
            optimized_drone_routes.append(truck_drone_routes)
            continue
        
        # Get detailed route data for truck segments
        truck_route_segments = []
        for i in range(len(truck_route) - 1):
            start_node = truck_route[i]
            end_node = truck_route[i + 1]
            
            start_lat, start_lon = lat_coords[start_node], lon_coords[start_node]
            end_lat, end_lon = lat_coords[end_node], lon_coords[end_node]
            
            route_data = get_route_data(start_lat, start_lon, end_lat, end_lon)
            truck_route_segments.append(route_data)
        
        optimized_truck_drones = []
        
        for drone_idx, drone_trips in enumerate(truck_drone_routes):
            optimized_drone_trips = []
            
            for trip_idx, trip in enumerate(drone_trips):
                if not trip or len(trip) < 3:  # Need takeoff, customer, landing
                    optimized_drone_trips.append(trip)
                    continue
                
                # Extract customer node (middle of trip)
                customer_node = None
                for node in trip[1:-1]:  # Exclude takeoff/landing points
                    if isinstance(node, (int, float)) and not isinstance(node, str):
                        customer_node = int(node)
                        break
                
                if customer_node is None:
                    optimized_drone_trips.append(trip)
                    continue
                
                customer_lat = lat_coords[customer_node]
                customer_lon = lon_coords[customer_node]
                
                print(f"  Optimizing Drone {drone_idx + 1}, Trip {trip_idx + 1} to customer {customer_node}")
                
                # Set up optimization problem
                problem = DroneOptimizationProblem(
                    truck_route_segments, customer_lat, customer_lon,
                    truck_speed, drone_speed
                )
                
                # Create Optuna study
                study = optuna.create_study(direction='minimize')
                study.optimize(problem.objective, n_trials=100, show_progress_bar=False)
                
                if study.best_value == float('inf'):
                    print(f"    No feasible solution found, keeping original route")
                    optimized_drone_trips.append(trip)
                    continue
                
                # Extract optimal takeoff and landing positions
                best_takeoff_dist = study.best_params['takeoff_distance']
                best_landing_dist = study.best_params['landing_distance']
                
                takeoff_pos = problem.get_position_on_route(best_takeoff_dist)
                landing_pos = problem.get_position_on_route(best_landing_dist)
                
                if takeoff_pos and landing_pos:
                    takeoff_lat, takeoff_lon = takeoff_pos
                    landing_lat, landing_lon = landing_pos
                    
                    # Create optimized trip
                    new_takeoff = f"V({takeoff_lat:.6f},{takeoff_lon:.6f})"
                    new_landing = f"V({landing_lat:.6f},{landing_lon:.6f})"
                    
                    optimized_trip = [new_takeoff, customer_node, new_landing]
                    optimized_drone_trips.append(optimized_trip)
                    
                    # Calculate performance metrics
                    drone_time, truck_time = problem.calculate_drone_mission_time(
                        best_takeoff_dist, best_landing_dist)
                    
                    print(f"    ✅ Optimized: Drone time {drone_time:.2f}, Truck time {truck_time:.2f}")
                    print(f"    Takeoff: ({takeoff_lat:.4f}, {takeoff_lon:.4f})")
                    print(f"    Landing: ({landing_lat:.4f}, {landing_lon:.4f})")
                else:
                    print(f"    ❌ Failed to get positions, keeping original route")
                    optimized_drone_trips.append(trip)
            
            optimized_truck_drones.append(optimized_drone_trips)
        
        optimized_drone_routes.append(optimized_truck_drones)
    
    print("\n=== Optimization Complete ===")
    return optimized_drone_routes
