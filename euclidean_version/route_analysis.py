"""
Route data extraction for detailed waypoint analysis - Euclidean version
"""
from distance_utils import euclidean_distance


class RouteSegment:
    """Represents a segment of a route with timing information"""
    def __init__(self, start_lat, start_lon, end_lat, end_lon, distance=None, cumulative_distance=0):
        self.start_lat = start_lat
        self.start_lon = start_lon
        self.end_lat = end_lat
        self.end_lon = end_lon
        self.distance = distance if distance is not None else euclidean_distance(start_lat, start_lon, end_lat, end_lon)
        self.cumulative_distance = cumulative_distance
        self.start_time = cumulative_distance  # Assuming unit speed
        
    def get_position_at_time(self, time, truck_speed=1.0):
        """Get truck position at given time on this segment"""
        distance_traveled = time * truck_speed
        
        if distance_traveled <= self.cumulative_distance:
            return None  # Truck hasn't reached this segment yet
        
        segment_travel_distance = distance_traveled - self.cumulative_distance
        
        if segment_travel_distance >= self.distance:
            return None  # Truck has passed this segment
        
        # Calculate position along segment
        progress = segment_travel_distance / self.distance if self.distance > 0 else 0
        
        lat = self.start_lat + progress * (self.end_lat - self.start_lat)
        lon = self.start_lon + progress * (self.end_lon - self.start_lon)
        
        return (lat, lon)


class RouteData:
    """Complete route data with timing for Euclidean distances"""
    def __init__(self, start_node, end_node, start_lat, start_lon, end_lat, end_lon):
        self.start_node = start_node
        self.end_node = end_node
        self.start_lat = start_lat
        self.start_lon = start_lon
        self.end_lat = end_lat
        self.end_lon = end_lon
        self.distance = euclidean_distance(start_lat, start_lon, end_lat, end_lon)
        self.start_time = 0


def get_route_data(truck_route, lat_coords, lon_coords, truck_speed=1.0):
    """Get a list of RouteData objects for each segment in a truck route"""
    route_segments = []
    cumulative_distance = 0
    
    for i in range(len(truck_route) - 1):
        start_node = truck_route[i]
        end_node = truck_route[i+1]
        
        start_lat = lat_coords[start_node]
        start_lon = lon_coords[start_node]
        end_lat = lat_coords[end_node]
        end_lon = lon_coords[end_node]
        
        distance = euclidean_distance(start_lat, start_lon, end_lat, end_lon)
        
        segment = RouteData(start_node, end_node, start_lat, start_lon, end_lat, end_lon)
        segment.distance = distance
        segment.start_time = cumulative_distance / truck_speed
        
        route_segments.append(segment)
        cumulative_distance += distance
    
    return route_segments
