"""
Google Maps API caching system to avoid repeated API calls and save costs
"""
import os
import json
import hashlib
from distance_utils import haversine_distance
from file_utils import GOOGLE_MAPS_API_KEY
import requests
import time


class GoogleMapsCache:
    def __init__(self, cache_dir="google_maps_cache"):
        """
        Initialize the cache system
        
        Args:
            cache_dir (str): Directory to store cache files
        """
        self.cache_dir = cache_dir
        self.cache_file = None
        self.cache_data = {}
        
        # Create cache directory if it doesn't exist
        if not os.path.exists(self.cache_dir):
            os.makedirs(self.cache_dir)
    
    def _generate_cache_filename(self, lat_coords, lon_coords):
        """
        Generate a unique cache filename based on coordinates
        
        Args:
            lat_coords: Array of latitude coordinates
            lon_coords: Array of longitude coordinates
            
        Returns:
            str: Cache filename
        """
        # Create a hash of the coordinates to generate unique filename
        coords_str = f"{lat_coords.tolist()}{lon_coords.tolist()}"
        hash_obj = hashlib.md5(coords_str.encode())
        hash_hex = hash_obj.hexdigest()
        return f"gmaps_cache_{hash_hex}.json"
    
    def _load_cache(self, cache_filename):
        """
        Load existing cache from file
        
        Args:
            cache_filename (str): Name of cache file
        """
        self.cache_file = os.path.join(self.cache_dir, cache_filename)
        
        if os.path.exists(self.cache_file):
            try:
                with open(self.cache_file, 'r') as f:
                    self.cache_data = json.load(f)
                print(f"Loaded Google Maps cache from {self.cache_file}")
                print(f"Cache contains {len(self.cache_data)} distance pairs")
            except Exception as e:
                print(f"Error loading cache file: {e}")
                self.cache_data = {}
        else:
            self.cache_data = {}
            print(f"No existing cache found. Will create new cache file: {self.cache_file}")
    
    def _save_cache(self):
        """
        Save current cache data to file
        """
        try:
            with open(self.cache_file, 'w') as f:
                json.dump(self.cache_data, f, indent=2)
            print(f"Saved Google Maps cache to {self.cache_file}")
            print(f"Cache now contains {len(self.cache_data)} distance pairs")
        except Exception as e:
            print(f"Error saving cache file: {e}")
    
    def _create_cache_key(self, lat1, lon1, lat2, lon2):
        """
        Create a unique key for coordinate pair
        
        Args:
            lat1, lon1, lat2, lon2: Coordinates
            
        Returns:
            str: Cache key
        """
        # Create a consistent key regardless of order
        point1 = f"{lat1:.6f},{lon1:.6f}"
        point2 = f"{lat2:.6f},{lon2:.6f}"
        
        # Sort to ensure same key for A->B and B->A
        if point1 < point2:
            return f"{point1}-{point2}"
        else:
            return f"{point2}-{point1}"
    
    def get_distance(self, lat1, lon1, lat2, lon2):
        """
        Get distance between two points, using cache if available
        
        Args:
            lat1, lon1, lat2, lon2: Coordinates
            
        Returns:
            float: Distance in kilometers
        """
        cache_key = self._create_cache_key(lat1, lon1, lat2, lon2)
        
        # Check if distance is in cache
        if cache_key in self.cache_data:
            return self.cache_data[cache_key]
        
        # If not in cache, call Google Maps API
        print(f"Cache miss for {cache_key} - calling Google Maps API")
        distance = self._call_google_maps_api(lat1, lon1, lat2, lon2)
        
        # Store in cache
        self.cache_data[cache_key] = distance
        
        return distance
    
    def _call_google_maps_api(self, lat1, lon1, lat2, lon2):
        """
        Call Google Maps API to get actual distance
        
        Args:
            lat1, lon1, lat2, lon2: Coordinates
            
        Returns:
            float: Distance in kilometers
        """
        url = f"https://maps.googleapis.com/maps/api/directions/json?origin={lat1},{lon1}&destination={lat2},{lon2}&mode=driving&key={GOOGLE_MAPS_API_KEY}"
        
        try:
            response = requests.get(url)
            data = response.json()
            
            if data["status"] == "OK":
                distance_m = data["routes"][0]["legs"][0]["distance"]["value"]
                distance_km = distance_m / 1000.0
                print(f"Google Maps API: {distance_km:.2f} km for {lat1:.4f},{lon1:.4f} -> {lat2:.4f},{lon2:.4f}")
                time.sleep(0.2)  # Rate limiting
                return distance_km
            else:
                print(f"Google Maps API request failed with status: {data['status']}")
                # Fallback to Haversine distance
                fallback_distance = haversine_distance(lat1, lon1, lat2, lon2)
                print(f"Using Haversine fallback: {fallback_distance:.2f} km")
                return fallback_distance
                
        except Exception as e:
            print(f"Error accessing Google Maps API: {e}")
            # Fallback to Haversine distance
            fallback_distance = haversine_distance(lat1, lon1, lat2, lon2)
            print(f"Using Haversine fallback: {fallback_distance:.2f} km")
            return fallback_distance
    
    def initialize_for_coordinates(self, lat_coords, lon_coords):
        """
        Initialize cache for a specific set of coordinates
        
        Args:
            lat_coords: Array of latitude coordinates
            lon_coords: Array of longitude coordinates
        """
        cache_filename = self._generate_cache_filename(lat_coords, lon_coords)
        self._load_cache(cache_filename)
    
    def finalize_cache(self):
        """
        Save cache and show statistics
        """
        if self.cache_file:
            self._save_cache()
            print(f"\n=== Google Maps Cache Statistics ===")
            print(f"Cache file: {self.cache_file}")
            print(f"Total cached distances: {len(self.cache_data)}")
            print(f"Cache file size: {os.path.getsize(self.cache_file) / 1024:.1f} KB")
            print("=====================================\n")


# Global cache instance
_cache_instance = None

def get_cache_instance():
    """Get or create the global cache instance"""
    global _cache_instance
    if _cache_instance is None:
        _cache_instance = GoogleMapsCache()
    return _cache_instance
