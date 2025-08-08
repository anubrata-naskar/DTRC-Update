"""
Route optimization utilities including 2-opt, relocate, swap moves, and Christofides
"""
import networkx as nx
from networkx.algorithms.approximation import traveling_salesman_problem
from networkx.algorithms.approximation import christofides


def calculate_route_cost(route, dist_matrix):
    cost = 0
    for i in range(len(route) - 1):
        cost += dist_matrix.iloc[route[i], route[i+1]]
    cost += dist_matrix.iloc[route[-1], 0]     
    return cost


def two_opt(route, dist_matrix):
    best = route
    improved = True
    while improved:
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route)):
                if j - i == 1: continue
                new_route = route[:i] + route[i:j][::-1] + route[j:]
                if calculate_route_cost(new_route, dist_matrix) < calculate_route_cost(best, dist_matrix):
                    best = new_route
                    improved = True
        route = best
    return best


def simple_relocate(route, dist_matrix):
    best = route
    for i in range(1, len(route) - 1):
        for j in range(1, len(route)):
            if i == j: continue
            new_route = route[:i] + route[i+1:j] + [route[i]] + route[j:]
            if calculate_route_cost(new_route, dist_matrix) < calculate_route_cost(best, dist_matrix):
                best = new_route
    return best


def swap_move(route, dist_matrix):
    best = route    
    for i in range(1, len(route) - 1):
        for j in range(i + 1, len(route)):
            new_route = route[:i] + [route[j]] + route[i+1:j] + [route[i]] + route[j+1:]
            if calculate_route_cost(new_route, dist_matrix) < calculate_route_cost(best, dist_matrix):
                best = new_route
    return best


def christofides_route(route, dist_matrix):
    G = nx.Graph()
    
    for i in route:
        G.add_node(i)

    for i in route:
        for j in route:
            if i != j:
                G.add_edge(i, j, weight=dist_matrix.iloc[i, j])

    tsp_path = traveling_salesman_problem(G, cycle=True, method=christofides)

    if tsp_path[0] != 0:
        zero_index = tsp_path.index(0)
        tsp_path = tsp_path[zero_index:] + tsp_path[1:zero_index] + [0]

    return tsp_path
