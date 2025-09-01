# DTRC Euclidean Version

This is a modified version of the Drone-Truck Routing Coordination (DTRC) algorithm that uses pure Euclidean distances for all calculations instead of Google Maps API.

## Key Features

- Uses Euclidean distance for all distance calculations
- No Google Maps API dependency
- Fully compatible with standard CVRP format files
- Includes visualization using matplotlib

## How to Use

1. Place your CVRP format file in this folder
2. Run `main.py` to execute the algorithm
3. View the generated route visualizations and cost calculations

## CVRP File Format

The algorithm expects files in the standard CVRP format, for example:

```
NAME : A-n33-k6
COMMENT : (Augerat et al, No of trucks: 6, Optimal value: 742)
TYPE : CVRP
DIMENSION : 33
EDGE_WEIGHT_TYPE : EUC_2D 
CAPACITY : 100
NODE_COORD_SECTION 
 1 34 31
 2 45 55
...
DEMAND_SECTION 
1 0 
2 26 
...
DEPOT_SECTION 
 1  
 -1  
EOF
```

Where:
- Node 1 is the depot
- Nodes 2-33 are delivery points
- k6 indicates there are 6 trucks
- Coordinates are in the format: Node_ID X_coordinate Y_coordinate
- Demand section shows: Node_ID Demand_Amount
- Capacity value is the truck capacity
