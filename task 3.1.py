# Import all utilities
from warehouse_utils import *

# Compute all shortest paths
all_paths = compute_all_shortest_paths()

print("--- Dijkstra's Algorithm Paths Computed ---")

# --- Example Demonstrations ---
start_example_1, end_example_1 = 1, 40 
# Path example: 1,2,3,4,10,16,22, 28,34,40 [cite: 147]
path_1_to_40 = all_paths.get(start_example_1, {}).get(end_example_1)

start_example_2, end_example_2 = 1, 15 # Path that needs to route around obstacles 8, 9
path_1_to_15 = all_paths.get(start_example_2, {}).get(end_example_2)

start_example_3, end_example_3 = 28, 42 # Path routed near obstacle 29, 35
path_28_to_42 = all_paths.get(start_example_3, {}).get(end_example_3)


print(f"\nExample 1: Path from {start_example_1} to {end_example_1}")
print(f"Path: {path_1_to_40}")
print(f"Length: {len(path_1_to_40) - 1 if path_1_to_40 else 'N/A'}")

print(f"\nExample 2: Path from {start_example_2} to {end_example_2} (Routing around {STATIC_OBSTACLES.intersection({8, 9})})")
print(f"Path: {path_1_to_15}")
print(f"Length: {len(path_1_to_15) - 1 if path_1_to_15 else 'N/A'}")

print(f"\nExample 3: Path from {start_example_3} to {end_example_3} (Routing around {STATIC_OBSTACLES.intersection({29, 35})})")
print(f"Path: {path_28_to_42}")
print(f"Length: {len(path_28_to_42) - 1 if path_28_to_42 else 'N/A'}")

print(f"\nStatic Obstacles (Shelfs): {STATIC_OBSTACLES}")