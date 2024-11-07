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
    "YYZ": (43.6777, -79.6248), "ORD": (41.9742, -87.9073), "DFW": (32.8998, -97.0403),
    "MIA": (25.7959, -80.2870), "ATL": (33.6407, -84.4277), "SEA": (47.4502, -122.3088),
    "DEN": (39.8561, -104.6737), "YVR": (49.1951, -123.1779), "MEX": (19.4361, -99.0719),
    
    # Europe
    "LHR": (51.4700, -0.4543), "CDG": (49.0097, 2.5479), "FRA": (50.0379, 8.5622),
    "IST": (41.2753, 28.7519), "MAD": (40.4983, -3.5676), "FCO": (41.8003, 12.2389),
    "AMS": (52.3105, 4.7683), "ZRH": (47.4506, 8.5610), "VIE": (48.1103, 16.5697),
    "BRU": (50.9010, 4.4844), "CPH": (55.6180, 12.6560), "HEL": (60.3172, 24.9633),
    "DUB": (53.4273, -6.2436), "OSL": (60.1976, 11.1004), "ARN": (59.6498, 17.9233),
    "LGW": (51.1537, -0.1821), "ATH": (37.9364, 23.9445), "BCN": (41.2974, 2.0833),
    "MUC": (48.3538, 11.7861), "GOH": (64.1902, -51.6787), "KEF": (63.9815, -22.6282),
    
    # Africa
    "JNB": (-26.1367, 28.2460), "CPT": (-33.9706, 18.6016), "LOS": (6.5775, 3.3212),
    "CAI": (30.1219, 31.4054), "NBO": (-1.3192, 36.9275), "ADD": (8.9783, 38.7993),
    "CMN": (33.3675, -7.5899), "DSS": (14.6711, -17.0733), "LAD": (-8.8575, 13.2311),
    "ACC": (5.6052, -0.1682), "ABV": (9.0066, 7.2632), "ALG": (36.6964, 3.2155),
    "ROB": (6.2339, -10.3623), "FIH": (-4.3858, 15.4446), "DAR": (-6.8781, 39.2026),
    "MPM": (-25.9207, 32.5732), "DKR": (14.7400, -17.4902), "NDJ": (12.1348, 15.0346),
    "HRE": (-17.9318, 31.0928), "BGF": (4.3983, 18.5188), "EBB": (0.0424, 32.4435),
    "BKO": (12.5372, -7.9499), "LFW": (6.1656, 1.2545), "COO": (6.3572, 2.3844),
    "NIM": (13.4826, 2.1836), "OUA": (12.3532, -1.5128), "MRU": (-20.4302, 57.6836),
    "SEZ": (-4.6743, 55.5218), "HGA": (9.5182, 44.0888), "FBM": (-11.5903, 27.5309),
    
    # South America
    "GRU": (-23.4321, -46.4694), "EZE": (-34.8222, -58.5358), "BOG": (4.7016, -74.1469),
    "LIM": (-12.0219, -77.1143), "SCL": (-33.3930, -70.7858), "GIG": (-22.8110, -43.2506),
    "MVD": (-34.8384, -56.0308), "UIO": (-0.1250, -78.3586), "CCS": (10.6031, -66.9906),
    "PTY": (9.0714, -79.3835), "NAT": (-5.9067, -35.2072), "AEP": (-34.5592, -58.4156),
    "POA": (-29.9935, -51.1754), "CWB": (-25.5285, -49.1758), "REC": (-8.1254, -34.9233),
    "BSB": (-15.8711, -47.9186), "ASU": (-25.2396, -57.5191), "GYE": (-2.1574, -79.8836),
    "CLO": (3.5432, -76.3816), "LPB": (-16.5133, -68.1923), "VLN": (10.1599, -67.9283),
    "BLA": (10.1111, -64.6886), "CUZ": (-13.5357, -71.9384),
    
    # Asia-Pacific (for connectivity)
    "DEL": (28.5562, 77.1000), "BOM": (19.0896, 72.8656), "DXB": (25.2532, 55.3657),
    "JED": (21.6796, 39.1565), "DOH": (25.2769, 51.5200), "BKK": (13.6899, 100.7501),
    "HKG": (22.3080, 113.9185), "PEK": (40.0801, 116.5846), "ICN": (37.4602, 126.4407),
    "NRT": (35.7720, 140.3929), "PVG": (31.1443, 121.8083), "SYD": (-33.9399, 151.1753),
    "KUL": (2.7456, 101.7072), "TPE": (25.0794, 121.2340), "SIN": (1.3644, 103.9915)
}



def haversine(coord1, coord2):
    # Radius of Earth in kilometers
    R = 6371.0
    lat1 = radians(coord1[0])
    lon1 = radians(coord1[1])
    lat2 = radians(coord2[0])
    lon2 = radians(coord2[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c * 0.621371  # Convert to miles
    return distance


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

# Dijkstra's algorithm to find the shortest path by travel time
def dijkstra_shortest_time(graph, source, target):
    path = nx.dijkstra_path(graph, source=source, target=target, weight="weight")
    total_time = nx.dijkstra_path_length(graph, source=source, target=target, weight="weight")
    return path, total_time

# Run the Dijkstra algorithm on the modified graph
flight_path, total_time = dijkstra_shortest_time(G, source="JFK", target="SIN")

# Print out the flight path and total travel time
print("\nFlight Path Summary:")
for i in range(len(flight_path) - 1):
    start = flight_path[i]
    end = flight_path[i + 1]
    print(f"{start} -> {end}")

print(f"\nTotal travel time: {total_time:.2f} hours")

# Plot the graph and highlight the shortest path
plt.figure(figsize=(14, 10))

# Plot all nodes and edges
pos = {airport: (airport_coords[airport][1], airport_coords[airport][0]) for airport in airport_coords.keys()}
nx.draw_networkx_nodes(G, pos, node_size=50, node_color="blue")
nx.draw_networkx_edges(G, pos, edgelist=G.edges, edge_color="gray", alpha=0.5)

# Highlight the shortest path in red
path_edges = list(zip(flight_path[:-1], flight_path[1:]))
nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color="red", width=2)
nx.draw_networkx_labels(G, pos, font_size=8, font_color="black")

plt.title("Flight Path from JFK to SIN")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.show()
