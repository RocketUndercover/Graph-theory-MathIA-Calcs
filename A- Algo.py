import networkx as nx
import matplotlib.pyplot as plt
from itertools import permutations
from math import radians, sin, cos, sqrt, atan2

# Aircraft specifications
fuel_burn_rate = 2.4  # pounds per mile
max_fuel_capacity = 5854  # pounds of fuel
max_range = max_fuel_capacity / fuel_burn_rate  # maximum range in miles

# Average speed of the aircraft
ground_speed_knots = 430  # knots
average_speed_kmh = ground_speed_knots * 1.852  # 1 knot = 1.852 km/h

# Latitude and longitude data for each airport
airport_coords = {
    # North America
    "JFK": (40.6413, -73.7781), "LAX": (33.9416, -118.4085), "SFO": (37.7749, -122.4194),
    # Asia-Pacific (for connectivity)
    "DEL": (28.5562, 77.1000), "SIN": (1.3644, 103.9915)
    # Add more airports here as needed
}


def haversine(coord1, coord2):
    R = 6371.0  # Radius of Earth in kilometers
    lat1, lon1 = radians(coord1[0]), radians(coord1[1])
    lat2, lon2 = radians(coord2[0]), radians(coord2[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1

    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance_km = R * c
    return distance_km * 0.621371  # Convert to miles


# Create a directed graph
G = nx.DiGraph()

# Add edges between airports if within the maximum range
for (src, dest) in permutations(airport_coords.keys(), 2):
    if src == "JFK" and dest == "SIN":
        continue  # Skip direct flight from JFK to SIN

    distance = haversine(airport_coords[src], airport_coords[dest])

    # Only add edge if within maximum range
    if distance <= max_range:
        time_hours = distance / (average_speed_kmh * 0.621371)  # Convert speed to miles per hour
        G.add_edge(src, dest, weight=time_hours, distance=distance)


# A* algorithm to find the shortest path by travel time
def a_star_shortest_time(graph, source, target):
    def heuristic(node):
        # Estimate remaining time to target based on Haversine distance
        distance_to_target = haversine(airport_coords[node], airport_coords[target])
        estimated_time = distance_to_target / (average_speed_kmh * 0.621371)
        return estimated_time

    open_set = {source}
    came_from = {}
    g_score = {node: float('inf') for node in graph.nodes}
    g_score[source] = 0
    f_score = {node: float('inf') for node in graph.nodes}
    f_score[source] = heuristic(source)

    # Initialize priority queue with (f_score, node) tuples
    priority_queue = [(f_score[source], source)]

    while open_set:
        current = min(priority_queue, key=lambda x: x[0])[1]
        priority_queue = [item for item in priority_queue if item[1] != current]

        if current == target:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(source)
            return path[::-1], g_score[target]  # Return the path and total travel time

        open_set.remove(current)

        for neighbor in graph.neighbors(current):
            tentative_g_score = g_score[current] + graph[current][neighbor]['weight']
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor)

                if neighbor not in open_set:
                    open_set.add(neighbor)
                    priority_queue.append((f_score[neighbor], neighbor))

    return None, float('inf')  # Return None if no path found


# Run the A* algorithm on the modified graph
flight_path, total_time = a_star_shortest_time(G, source="JFK", target="SIN")

# Print out the flight path and total travel time
print("\nFlight Path Summary:")
if flight_path:
    for i in range(len(flight_path) - 1):
        start = flight_path[i]
        end = flight_path[i + 1]
        print(f"{start} -> {end}")
    print(f"\nTotal travel time: {total_time:.2f} hours")
else:
    print("No feasible path found.")

# Plot the graph and highlight the shortest path
plt.figure(figsize=(14, 10))

# Plot all nodes and edges
pos = {airport: (airport_coords[airport][1], airport_coords[airport][0]) for airport in airport_coords.keys()}
nx.draw_networkx_nodes(G, pos, node_size=50, node_color="blue")
nx.draw_networkx_edges(G, pos, edgelist=G.edges, edge_color="gray", alpha=0.5)

# Highlight the shortest path in red
path_edges = list(zip(flight_path[:-1], flight_path[1:])) if flight_path else []
nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color="red", width=2)
nx.draw_networkx_labels(G, pos, font_size=8, font_color="black")

plt.title("Flight Path from JFK to SIN")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.show()
