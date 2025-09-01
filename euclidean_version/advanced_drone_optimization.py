"""
Advanced drone takeoff/landing optimization using Optuna and detailed route analysis
"""
import optuna
import numpy as np
from route_analysis import get_route_data
from distance_utils import euclidean_distance


class DroneOptimizationProblem:
    """Represents a drone optimization problem with timing constraints"""
    
    def __init__(self, truck_route_segments, customer_lat, customer_lon, 
                 truck_speed=1.0, drone_speed=1.5):
        self.truck_route_segments = truck_route_segments  # List of RouteData objects
        self.customer_lat = customer_lat
        self.customer_lon = customer_lon
        self.truck_speed = truck_speed
        self.drone_speed = drone_speed
        
    def calculate_truck_time(self, truck_idx, truck_point):
        """Calculate time for truck to reach a point along its route"""
        if truck_idx >= len(self.truck_route_segments):
            return float('inf')
            
        segment = self.truck_route_segments[truck_idx]
        
        # Calculate position along segment (0 to 1)
        start_lat, start_lon = segment.start_lat, segment.start_lon
        end_lat, end_lon = segment.end_lat, segment.end_lon
        
        t = truck_point  # Position along segment (0 to 1)
        
        # Calculate distance from start of segment to this point
        partial_segment_dist = segment.distance * t
        
        # Time is distance / speed
        segment_time = partial_segment_dist / self.truck_speed
        
        # Add accumulated time to reach the start of this segment
        total_time = segment.start_time + segment_time
        
        return total_time
    
    def calculate_drone_time(self, takeoff_segment_idx, takeoff_point, 
                             landing_segment_idx, landing_point):
        """Calculate total drone flight time for a trip"""
        
        # Calculate takeoff location
        takeoff_segment = self.truck_route_segments[takeoff_segment_idx]
        takeoff_lat = takeoff_segment.start_lat + takeoff_point * (takeoff_segment.end_lat - takeoff_segment.start_lat)
        takeoff_lon = takeoff_segment.start_lon + takeoff_point * (takeoff_segment.end_lon - takeoff_segment.start_lon)
        
        # Calculate landing location
        landing_segment = self.truck_route_segments[landing_segment_idx]
        landing_lat = landing_segment.start_lat + landing_point * (landing_segment.end_lat - landing_segment.start_lat)
        landing_lon = landing_segment.start_lon + landing_point * (landing_segment.end_lon - landing_segment.start_lon)
        
        # Calculate flight distances
        to_customer_dist = euclidean_distance(takeoff_lat, takeoff_lon, 
                                             self.customer_lat, self.customer_lon)
        
        from_customer_dist = euclidean_distance(self.customer_lat, self.customer_lon,
                                              landing_lat, landing_lon)
        
        # Calculate flight time
        flight_time = (to_customer_dist + from_customer_dist) / self.drone_speed
        
        return flight_time
    
    def evaluate_solution(self, takeoff_segment_idx, takeoff_point, landing_segment_idx, landing_point):
        """Evaluate a solution based on times and constraints"""
        
        if takeoff_segment_idx >= landing_segment_idx and not (takeoff_segment_idx == landing_segment_idx and takeoff_point < landing_point):
            return float('inf')  # Invalid: takeoff must be before landing
        
        # Calculate truck times at takeoff and landing
        takeoff_time = self.calculate_truck_time(takeoff_segment_idx, takeoff_point)
        landing_time = self.calculate_truck_time(landing_segment_idx, landing_point)
        
        # Calculate drone flight time
        drone_time = self.calculate_drone_time(takeoff_segment_idx, takeoff_point,
                                               landing_segment_idx, landing_point)
        
        # Truck launch time + drone flight time should be <= truck landing time
        if takeoff_time + drone_time > landing_time:
            return float('inf')  # Invalid: drone would arrive before truck
            
        # We want to minimize total operational time for this delivery
        objective = landing_time
        
        return objective


def optimize_drone_takeoff_landing_advanced(truck_routes, drone_routes, distances_df,
                                           lat_coords, lon_coords, demands,
                                           truck_speed=1.0, drone_speed=1.5):
    """
    Optimizes drone takeoff and landing points using advanced route analysis
    """
    optimized_drone_routes = []
    
    for truck_idx, truck_route in enumerate(truck_routes):
        # Analyze the truck route to get timing information
        route_segments = get_route_data(truck_route, lat_coords, lon_coords, truck_speed)
        
        truck_drone_routes = []
        
        for drone_idx, drone_route_list in enumerate(drone_routes[truck_idx]):
            optimized_drone_route_list = []
            
            for route in drone_route_list:
                if not route:
                    optimized_drone_route_list.append([])
                    continue
                    
                delivery_nodes = [node for node in route[1:-1] 
                                 if not (isinstance(node, str) and node.startswith('V('))]
                
                if not delivery_nodes:
                    optimized_drone_route_list.append(route)
                    continue
                    
                # For simplicity, consider only the first delivery node
                customer_node = delivery_nodes[0]
                customer_lat = lat_coords[customer_node]
                customer_lon = lon_coords[customer_node]
                
                # Create the optimization problem
                problem = DroneOptimizationProblem(
                    route_segments, customer_lat, customer_lon, 
                    truck_speed, drone_speed
                )
                
                # Use Optuna to find the best takeoff/landing points
                def objective(trial):
                    takeoff_segment_idx = trial.suggest_int('takeoff_segment_idx', 0, max(0, len(route_segments) - 1))
                    takeoff_point = trial.suggest_float('takeoff_point', 0, 1)
                    
                    landing_segment_idx_max = min(len(route_segments) - 1, takeoff_segment_idx + 3)
                    landing_segment_idx = trial.suggest_int('landing_segment_idx', takeoff_segment_idx, landing_segment_idx_max)
                    
                    if landing_segment_idx == takeoff_segment_idx:
                        landing_point = trial.suggest_float('landing_point', takeoff_point, 1)
                    else:
                        landing_point = trial.suggest_float('landing_point', 0, 1)
                        
                    return problem.evaluate_solution(takeoff_segment_idx, takeoff_point, 
                                                    landing_segment_idx, landing_point)
                
                try:
                    study = optuna.create_study(direction='minimize')
                    study.optimize(objective, n_trials=50)
                    
                    best_params = study.best_params
                    takeoff_segment_idx = best_params['takeoff_segment_idx']
                    takeoff_point = best_params['takeoff_point']
                    landing_segment_idx = best_params['landing_segment_idx']
                    landing_point = best_params['landing_point']
                    
                    # Calculate optimal takeoff location
                    takeoff_segment = route_segments[takeoff_segment_idx]
                    takeoff_lat = takeoff_segment.start_lat + takeoff_point * (takeoff_segment.end_lat - takeoff_segment.start_lat)
                    takeoff_lon = takeoff_segment.start_lon + takeoff_point * (takeoff_segment.end_lon - takeoff_segment.start_lon)
                    
                    # Calculate optimal landing location
                    landing_segment = route_segments[landing_segment_idx]
                    landing_lat = landing_segment.start_lat + landing_point * (landing_segment.end_lat - landing_segment.start_lat)
                    landing_lon = landing_segment.start_lon + landing_point * (landing_segment.end_lon - landing_segment.start_lon)
                    
                    # Create optimized route
                    optimized_takeoff = f"V({takeoff_lat:.6f},{takeoff_lon:.6f})"
                    optimized_landing = f"V({landing_lat:.6f},{landing_lon:.6f})"
                    
                    optimized_route = [optimized_takeoff]
                    optimized_route.extend(delivery_nodes)
                    optimized_route.append(optimized_landing)
                    
                    optimized_drone_route_list.append(optimized_route)
                    
                except Exception as e:
                    print(f"Optimization failed for route: {e}")
                    optimized_drone_route_list.append(route)
            
            truck_drone_routes.append(optimized_drone_route_list)
        
        optimized_drone_routes.append(truck_drone_routes)
    
    return optimized_drone_routes
