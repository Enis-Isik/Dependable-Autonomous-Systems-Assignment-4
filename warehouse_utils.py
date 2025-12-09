import heapq
import math

# --- Grid Definitions ---
NUM_ROWS = 7
NUM_COLS = 6
TOTAL_POSITIONS = NUM_ROWS * NUM_COLS

# Static obstacles (shelfs) as 1-based indices
# Positions 8, 9, 29, 35 are occupied
STATIC_OBSTACLES = {8, 9, 29, 35}

# Generate a list of all valid positions (not obstacles)
FREE_POSITIONS = [i for i in range(1, TOTAL_POSITIONS + 1) if i not in STATIC_OBSTACLES]

# --- Helper Functions ---

# Convert (row, col) to a 1-based position index
def to_index(r, c):
    """Converts 0-based coordinates (row, col) to a 1-based position index."""
    if 0 <= r < NUM_ROWS and 0 <= c < NUM_COLS:
        return r * NUM_COLS + c + 1
    return None

# Convert 1-based position index to (row, col)
def to_coords(index):
    """Converts a 1-based position index to 0-based coordinates (row, col)."""
    if 1 <= index <= TOTAL_POSITIONS:
        zero_based_index = index - 1
        r = zero_based_index // NUM_COLS
        c = zero_based_index % NUM_COLS
        return r, c
    return None

# Function to get neighbors (valid, non-obstacle, right-angle moves)
def get_neighbors(position, obstacles=STATIC_OBSTACLES):
    """Returns a list of valid, non-obstacle, neighboring positions."""
    r, c = to_coords(position)
    neighbors = []
    
    # Possible moves: Up, Down, Left, Right
    potential_moves = [
        (r - 1, c), # Up
        (r + 1, c), # Down
        (r, c - 1), # Left
        (r, c + 1)  # Right
    ]
    
    for next_r, next_c in potential_moves:
        neighbor_index = to_index(next_r, next_c)
        if neighbor_index is not None and neighbor_index not in obstacles:
            neighbors.append(neighbor_index)
            
    return neighbors

# Dijkstra's Algorithm implementation to find predecessors for shortest path
def dijkstra_shortest_paths(start_node, free_positions=FREE_POSITIONS):
    """Implements Dijkstra's algorithm to find shortest path predecessors."""
    distances = {node: float('inf') for node in free_positions}
    distances[start_node] = 0
    predecessors = {node: None for node in free_positions}
    priority_queue = [(0, start_node)] # (distance, node)
    
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        
        if current_distance > distances[current_node]:
            continue
            
        for neighbor in get_neighbors(current_node, STATIC_OBSTACLES):
            weight = 1 
            distance = current_distance + weight
            
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                predecessors[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))
                
    return predecessors

# Reconstruct the path from predecessors map
def reconstruct_path(start_node, end_node, predecessors):
    """Reconstructs the path from a predecessors dictionary."""
    path = []
    current = end_node
    while current is not None:
        path.append(current)
        current = predecessors.get(current)
        
    if path and path[-1] == start_node:
        return path[::-1] # Reverse to get start -> end
    else:
        return []

# Function to compute and store all shortest paths
def compute_all_shortest_paths():
    """Computes and stores the shortest path between every pair of free positions."""
    all_paths = {}
    
    for start_pos in FREE_POSITIONS:
        predecessors = dijkstra_shortest_paths(start_pos)
        all_paths[start_pos] = {}
        for end_pos in FREE_POSITIONS:
            if start_pos != end_pos:
                path = reconstruct_path(start_pos, end_pos, predecessors)
                all_paths[start_pos][end_pos] = path
    return all_paths