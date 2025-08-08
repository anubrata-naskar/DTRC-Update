"""
Google Maps route data extraction for detailed waypoint analysis
"""
import requests
import polyline
from file_utils import GOOGLE_MAPS_API_KEY
from distance_utils import haversine_distance


class RouteSegment:
    """Represents a segment of a route with timing information"""
    def __init__(self, start_lat, start_lon, end_lat, end_lon, distance_km, cumulative_distance=0):
        self.start_lat = start_lat
        self.start_lon = start_lon
        self.end_lat = end_lat
        self.end_lon = end_lon
        self.distance_km = distance_km
        self.cumulative_distance = cumulative_distance
        
    def get_position_at_time(self, time, truck_speed=1.0):
        """Get truck position at given time on this segment"""
        distance_traveled = time * truck_speed
        
        if distance_traveled <= self.cumulative_distance:
            return None  # Truck hasn't reached this segment yet
        
        segment_travel_distance = distance_traveled - self.cumulative_distance
        
        if segment_travel_distance >= self.distance_km:
            return None  # Truck has passed this segment
        
        # Calculate position along segment
        progress = segment_travel_distance / self.distance_km if self.distance_km > 0 else 0
        
        lat = self.start_lat + progress * (self.end_lat - self.start_lat)
        lon = self.start_lon + progress * (self.end_lon - self.start_lon)
        
        return (lat, lon)
    
    def time_to_reach_start(self, truck_speed=1.0):
        """Time for truck to reach start of this segment"""
        return self.cumulative_distance / truck_speed
    
    def time_to_reach_end(self, truck_speed=1.0):
        """Time for truck to reach end of this segment"""
        return (self.cumulative_distance + self.distance_km) / truck_speed


class RouteData:
    """Complete route data with detailed waypoints and timing"""
    def __init__(self, start_lat, start_lon, end_lat, end_lon):
        self.start_lat = start_lat
        self.start_lon = start_lon
        self.end_lat = end_lat
        self.end_lon = end_lon
        self.segments = []
        self.total_distance = 0
        self.waypoints = []
        
    def load_from_google_maps(self):
        """Load detailed route data from Google Maps API"""
        url = "https://maps.googleapis.com/maps/api/directions/json"
        params = {
            'origin': f'{self.start_lat},{self.start_lon}',
            'destination': f'{self.end_lat},{self.end_lon}',
            'mode': 'driving',
            'key': GOOGLE_MAPS_API_KEY
        }
        
        try:
            response = requests.get(url, params=params)
            data = response.json()
            
            if data['status'] != 'OK':
                print(f"Google Maps API error: {data['status']}")
                self._create_fallback_route()
                return
            
            # Extract detailed route information
            route = data['routes'][0]
            leg = route['legs'][0]
            
            # Decode polyline to get detailed waypoints
            encoded_polyline = route['overview_polyline']['points']
            decoded_points = polyline.decode(encoded_polyline)
            
            self.waypoints = [(lat, lon) for lat, lon in decoded_points]
            
            # Create segments from waypoints
            cumulative_distance = 0
            
            for i in range(len(self.waypoints) - 1):
                start_lat, start_lon = self.waypoints[i]
                end_lat, end_lon = self.waypoints[i + 1]
                
                segment_distance = haversine_distance(start_lat, start_lon, end_lat, end_lon)
                
                segment = RouteSegment(
                    start_lat, start_lon, end_lat, end_lon,
                    segment_distance, cumulative_distance
                )
                
                self.segments.append(segment)
                cumulative_distance += segment_distance
            
            self.total_distance = cumulative_distance
            print(f"Loaded route with {len(self.segments)} segments, total distance: {self.total_distance:.2f} km")
            
        except Exception as e:
            print(f"Error loading Google Maps route: {e}")
            self._create_fallback_route()
    
    def _create_fallback_route(self):
        """Create simple fallback route if Google Maps fails"""
        distance = haversine_distance(self.start_lat, self.start_lon, self.end_lat, self.end_lon)
        segment = RouteSegment(
            self.start_lat, self.start_lon, self.end_lat, self.end_lon,
            distance, 0
        )
        self.segments = [segment]
        self.total_distance = distance
        self.waypoints = [(self.start_lat, self.start_lon), (self.end_lat, self.end_lon)]
    
    def get_truck_position_at_time(self, time, truck_speed=1.0):
        """Get truck position at given time"""
        for segment in self.segments:
            position = segment.get_position_at_time(time, truck_speed)
            if position:
                return position
        
        # If time is beyond route completion, return end position
        if time * truck_speed >= self.total_distance:
            return (self.end_lat, self.end_lon)
        
        return (self.start_lat, self.start_lon)
    
    def get_total_travel_time(self, truck_speed=1.0):
        """Get total time to complete route"""
        return self.total_distance / truck_speed
    
    def get_segments_in_time_range(self, start_time, end_time, truck_speed=1.0):
        """Get segments that truck traverses in given time range"""
        relevant_segments = []
        
        for segment in self.segments:
            segment_start_time = segment.time_to_reach_start(truck_speed)
            segment_end_time = segment.time_to_reach_end(truck_speed)
            
            # Check if segment overlaps with time range
            if segment_end_time >= start_time and segment_start_time <= end_time:
                relevant_segments.append(segment)
        
        return relevant_segments


def get_route_data(start_lat, start_lon, end_lat, end_lon, use_cache=True):
    """Get detailed route data between two points"""
    route = RouteData(start_lat, start_lon, end_lat, end_lon)
    
    if use_cache:
        # Check if we have cached route data
        from google_maps_cache import get_cache_instance
        cache = get_cache_instance()
        
        # For now, always load from API (can be enhanced to cache route data)
        route.load_from_google_maps()
    else:
        route.load_from_google_maps()
    
    return route
