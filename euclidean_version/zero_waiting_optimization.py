import numpy as np
from distance_utils import euclidean_distance
from route_analysis import get_route_data


def optimize_for_zero_waiting(truck_routes, drone_routes, distances_df, 
                              lat_coords, lon_coords, demands,
                              truck_speed=1.0, drone_speed=1.5, 
                              max_iterations=100, tolerance=0.01):
    optimized_routes = []
    print("\nZero-Waiting Optimization - Landing Point Changes:")
    print("-" * 80)
    for truck_idx, truck_route in enumerate(truck_routes):
        route_segments = get_route_data(truck_route, lat_coords, lon_coords, truck_speed)
        truck_optimized = []
        drone_count = 0
        for drone_route_list in drone_routes[truck_idx]:
            optimized_list = []
            for route in drone_route_list:
                if not route or len(route) < 3:
                    optimized_list.append(route)
                    continue
                takeoff = route[0]
                old_landing = route[-1]
                customers = [n for n in route[1:-1] if not isinstance(n, str)]
                if not customers:
                    optimized_list.append(route)
                    continue
                last_customer = customers[-1]
                takeoff_lat, takeoff_lon, takeoff_time = get_location_time(
                    takeoff, truck_route, lat_coords, lon_coords, distances_df, truck_speed)
                current_lat, current_lon = takeoff_lat, takeoff_lon
                total_drone_dist = 0
                for cust in customers:
                    dist = euclidean_distance(current_lat, current_lon, lat_coords[cust], lon_coords[cust])
                    total_drone_dist += dist
                    current_lat, current_lon = lat_coords[cust], lon_coords[cust]
                best_landing = find_best_landing(
                    route_segments, lat_coords[last_customer], lon_coords[last_customer],
                    takeoff_time, total_drone_dist, drone_speed, truck_speed)
                print(f"Truck {truck_idx} - Drone {drone_count}:")
                print(f"  Customers: {customers}")
                print(f"  Previous landing: {old_landing}")
                print(f"  New landing:      {best_landing}")
                if old_landing != best_landing:
                    print(f"  Status: CHANGED ✓")
                else:
                    print(f"  Status: UNCHANGED")
                drone_count += 1
                new_route = [takeoff] + customers + [best_landing]
                optimized_list.append(new_route)
            truck_optimized.append(optimized_list)
        optimized_routes.append(truck_optimized)
    print("-" * 80)
    return optimized_routes


def get_location_time(point, truck_route, lat_coords, lon_coords, distances_df, truck_speed):
    if isinstance(point, str) and point.startswith('V('):
        coords = point.strip('V()').split(',')
        return float(coords[0]), float(coords[1]), 0
    lat, lon = lat_coords[point], lon_coords[point]
    time = sum(distances_df.iloc[truck_route[i], truck_route[i+1]] / truck_speed 
               for i in range(truck_route.index(point))) if point in truck_route else 0
    return lat, lon, time


def find_best_landing(route_segments, customer_lat, customer_lon, 
                     takeoff_time, dist_to_customer, drone_speed, truck_speed):
    best_landing = None
    min_diff = float('inf')
    for segment in route_segments:
        for alpha in np.linspace(0, 1, 50):
            land_lat = segment.start_lat + alpha * (segment.end_lat - segment.start_lat)
            land_lon = segment.start_lon + alpha * (segment.end_lon - segment.start_lon)
            truck_time = segment.start_time + (segment.distance * alpha) / truck_speed
            dist_from_customer = euclidean_distance(customer_lat, customer_lon, land_lat, land_lon)
            drone_time = takeoff_time + (dist_to_customer + dist_from_customer) / drone_speed
            time_diff = abs(truck_time - drone_time)
            if time_diff < min_diff:
                min_diff = time_diff
                best_landing = f"V({land_lat:.6f},{land_lon:.6f})"
                if time_diff < 0.001:
                    return best_landing
    return best_landing if best_landing else "V(0,0)"


def verify_zero_waiting(truck_routes, drone_routes, distances_df, 
                       lat_coords, lon_coords, truck_speed=1.0, drone_speed=1.5):
    total_waiting = 0
    for truck_idx, truck_route in enumerate(truck_routes):
        for drone_route_list in drone_routes[truck_idx]:
            for route in drone_route_list:
                if not route or len(route) < 3:
                    continue
                takeoff = route[0]
                landing = route[-1]
                customers = [n for n in route[1:-1] if not isinstance(n, str)]
                if not customers:
                    continue
                takeoff_lat, takeoff_lon, t_takeoff = get_location_time(
                    takeoff, truck_route, lat_coords, lon_coords, distances_df, truck_speed)
                land_lat, land_lon, t_truck = get_location_time(
                    landing, truck_route, lat_coords, lon_coords, distances_df, truck_speed)
                current_lat, current_lon = takeoff_lat, takeoff_lon
                total_flight_time = 0
                for cust in customers:
                    dist = euclidean_distance(current_lat, current_lon, lat_coords[cust], lon_coords[cust])
                    total_flight_time += dist / drone_speed
                    current_lat, current_lon = lat_coords[cust], lon_coords[cust]
                dist_to_landing = euclidean_distance(current_lat, current_lon, land_lat, land_lon)
                total_flight_time += dist_to_landing / drone_speed
                t_drone = t_takeoff + total_flight_time
                time_diff = abs(t_drone - t_truck)
                if time_diff > 0.01:
                    print(f"  Truck {truck_idx}, Customers {customers}: Time difference = {time_diff:.4f} (Drone: {t_drone:.4f}, Truck: {t_truck:.4f})")
                    total_waiting += time_diff
    return total_waiting
