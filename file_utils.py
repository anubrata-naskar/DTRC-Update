"""
File utilities for reading CVRP files and configuration
"""
import os


def dynamic_battery_model(remaining_payload, distance_traveled=0):
    return (155885 / ((200 + remaining_payload) ** 1.5)) - distance_traveled


def read_cvrp_file(file_path):
    with open(file_path, "r") as file:
        lines = file.readlines()

    node_coords = []
    demands = {}
    num_trucks = None
    capacity = None
    section = None

    for line in lines:
        parts = line.strip().split()
        
        if not parts:
            continue

        if parts[0] == "COMMENT" and "No of trucks" in line:
            num_trucks = int(''.join(filter(str.isdigit, line.split("No of trucks:")[-1])))

        elif parts[0] == "CAPACITY":
            capacity = int(parts[-1])

        elif parts[0] == "NODE_COORD_SECTION":
            section = "NODE_COORD"

        elif parts[0] == "DEMAND_SECTION":
            section = "DEMAND"

        elif parts[0] == "DEPOT_SECTION":
            section = "DEPOT"

        elif section == "NODE_COORD":
            node_coords.append((int(parts[0]), float(parts[1]), float(parts[2])))

        elif section == "DEMAND":
            demands[int(parts[0])] = int(parts[1])

    import numpy as np
    node_coords = np.array(node_coords)
    lat_coords = node_coords[:, 1]
    lon_coords = node_coords[:, 2]

    return num_trucks, capacity, lat_coords, lon_coords, demands


# Configuration
GOOGLE_MAPS_API_KEY = os.getenv('API_KEY')
