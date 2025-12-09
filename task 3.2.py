# Import all utilities
from warehouse_utils import *

# Define direction mapping for proper boundary following (to keep track of robot's heading)
# (dr, dc): (change in row, change in col)
# N: (-1, 0), S: (1, 0), W: (0, -1), E: (0, 1)
DIRECTION_VECTORS = {
    (0, -1): 'N', (0, 1): 'S', (-1, 0): 'W', (1, 0): 'E'
}
# Priority for right-turn boundary following (relative to current direction)
# E.g., if moving E (0, 1), try S(1, 0), then E(0, 1), then N(-1, 0), then W(0, -1)
# Note: Python tuples are (r, c) or (dr, dc)
RIGHT_TURN_ORDER = {
    # Current Dir: [Turn Right, Go Straight, Turn Left, Turn Back]
    (0, 1): [(1, 0), (0, 1), (-1, 0), (0, -1)],  # E: [S, E, N, W]
    (0, -1): [(-1, 0), (0, -1), (1, 0), (0, 1)], # W: [N, W, S, E]
    (1, 0): [(0, -1), (1, 0), (0, 1), (-1, 0)],  # S: [W, S, E, N]
    (-1, 0): [(0, 1), (-1, 0), (0, -1), (1, 0)], # N: [E, N, W, S]
}

class Bug2Robot:
    def __init__(self, start_pos, goal_pos, precomputed_path, obstacles):
        self.current_pos = start_pos
        self.goal_pos = goal_pos
        self.precomputed_path = precomputed_path
        self.obstacles = obstacles # Static + Unexpected
        self.path_taken = [start_pos]
        self.state = "MOVE_TO_GOAL"
        self.m_line = self._calculate_m_line()
        self.hit_point = None
        self.hit_distance = float('inf')
        self.leave_point = None
        self.current_dir = None # Track robot's last movement vector (dr, dc)
        
    def _calculate_m_line(self):
        """Calculates the set of positions on the straight line (M-Line) from start to goal."""
        # ... (Same as before)
        start_r, start_c = to_coords(self.precomputed_path[0])
        goal_r, goal_c = to_coords(self.goal_pos)
        m_line_points = set()
        r, c = start_r, start_c
        
        while r != goal_r or c != goal_c:
            m_line_points.add(to_index(r, c))
            dr = 1 if goal_r > r else (-1 if goal_r < r else 0)
            dc = 1 if goal_c > c else (-1 if goal_c < c else 0)
            
            if abs(goal_r - r) >= abs(goal_c - c):
                r += dr
            else:
                c += dc
                
        m_line_points.add(self.goal_pos)
        return m_line_points

    def _is_blocked(self, position):
        """Checks if a position is an obstacle or outside the map boundary."""
        return position in self.obstacles or position not in range(1, TOTAL_POSITIONS + 1)

    def _dist_to_goal(self, pos):
        """Calculates Manhattan distance to the goal."""
        r, c = to_coords(pos)
        goal_r, goal_c = to_coords(self.goal_pos)
        return abs(goal_r - r) + abs(goal_c - c)

    def _find_boundary_move(self):
        """
        Implements a consistent right-turn wall-following rule.
        Requires tracking the current direction (self.current_dir).
        """
        r, c = to_coords(self.current_pos)
        
        # Determine the initial direction if just starting FOLLOW_BOUNDARY
        if self.current_dir is None:
            # Simple assumption: start by trying to move 'right' relative to the path
            # (16 -> 22 was blocked. Try 16 -> 17 (E) or 16 -> 15 (W))
            # Let's just pick the first non-blocked neighbor in order (E, S, W, N)
            initial_moves = [(0, 1), (1, 0), (0, -1), (-1, 0)]
            for dc, dr in initial_moves:
                 neighbor_index = to_index(r + dr, c + dc)
                 if neighbor_index is not None and not self._is_blocked(neighbor_index):
                    self.current_dir = (dr, dc)
                    return neighbor_index
            return None # Stuck
        
        # Use the stored direction to prioritize moves
        move_priorities = RIGHT_TURN_ORDER.get(self.current_dir)
        
        # 1. Try to turn Right (first in priority list)
        for dr, dc in move_priorities:
            neighbor_index = to_index(r + dr, c + dc)
            if neighbor_index is not None and not self._is_blocked(neighbor_index):
                # The move is clear, update direction and return
                self.current_dir = (dr, dc)
                return neighbor_index
        
        # Should not be reached if boundary is followable
        return None 

    def _get_move_to_goal(self):
        """Gets the next position on the precomputed path."""
        try:
            current_index = self.precomputed_path.index(self.current_pos)
            if current_index + 1 < len(self.precomputed_path):
                return self.precomputed_path[current_index + 1]
            else:
                return None
        except ValueError:
            # This happens if the robot has successfully left the boundary and is now *not* on the original path.
            # We must find the closest forward point on the path and update the index.
            
            # Since the robot has left the boundary at a point closer to the goal, 
            # we should look for the path point closest to the current position.
            
            min_dist = float('inf')
            next_path_pos = None
            
            # Search from the hit point index onward to find the closest path point
            # to transition back to the main route.
            start_index = self.precomputed_path.index(self.hit_point) if self.hit_point in self.precomputed_path else 0
            
            for i in range(start_index, len(self.precomputed_path)):
                path_pos = self.precomputed_path[i]
                r_r, c_r = to_coords(self.current_pos)
                r_p, c_p = to_coords(path_pos)
                dist = abs(r_r - r_p) + abs(c_r - c_p) # Manhattan distance
                
                if dist < min_dist:
                    min_dist = dist
                    next_path_pos = path_pos
                    self.path_index = i - 1 # Set index to the position *before* the next one we will target
            
            # If we successfully found a path point, we target the next one
            if next_path_pos:
                 return self.precomputed_path[self.path_index + 1]

            return None # Cannot find a path point


    def step(self):
        if self.current_pos == self.goal_pos:
            return True 
        
        next_pos = None
        
        if self.state == "MOVE_TO_GOAL":
            predicted_next_pos = self._get_move_to_goal()
            
            if predicted_next_pos is None:
                return True 
            
            # Check for unexpected obstacle at the next step 
            if self._is_blocked(predicted_next_pos):
                print(f"Robot detects unexpected obstacle at {predicted_next_pos} from {self.current_pos}. Initiating Bug2 boundary following.")
                self.state = "FOLLOW_BOUNDARY"
                self.hit_point = self.current_pos
                self.hit_distance = self._dist_to_goal(self.hit_point)
                self.current_dir = None # Reset direction for boundary initialization

                # Find the first valid move around the obstacle
                next_pos = self._find_boundary_move() 
                if next_pos is None:
                    print("Error: Completely surrounded by obstacles.")
                    self.state = "FAILURE"
                    return False
            else:
                # Update direction based on the planned move
                r_c, c_c = to_coords(self.current_pos)
                r_n, c_n = to_coords(predicted_next_pos)
                self.current_dir = (r_n - r_c, c_n - c_c)
                next_pos = predicted_next_pos
            
        elif self.state == "FOLLOW_BOUNDARY":
            
            # Find the next boundary move
            next_pos = self._find_boundary_move()
            
            if next_pos is None:
                print("Error: Stuck while following boundary.")
                self.state = "FAILURE"
                return False
                
            # Bug2 Leave Condition: Intersect M-Line AND distance to goal is less than at hit_point 
            current_distance_to_goal = self._dist_to_goal(next_pos)
            
            if next_pos in self.m_line and current_distance_to_goal < self.hit_distance:
                print(f"Robot intersects M-Line at {next_pos}, which is closer (dist {current_distance_to_goal}) to goal than hit point (dist {self.hit_distance}). Resuming MOVE_TO_GOAL.")
                self.state = "MOVE_TO_GOAL"
                self.leave_point = self.current_pos
            elif next_pos == self.hit_point and self.current_pos != self.hit_point and self._dist_to_goal(next_pos) >= self.hit_distance:
                # Returned to hit point without finding a better leave point
                print("Robot has returned to the hit point without finding a closer M-Line intersection. Obstacle completely blocks the path.")
                self.state = "FAILURE"
                return False

        if next_pos and self.state != "FAILURE":
            # Update path index only when moving along the precomputed path
            if self.state == "MOVE_TO_GOAL" and next_pos in self.precomputed_path:
                try:
                    self.path_index = self.precomputed_path.index(next_pos)
                except ValueError:
                    # Should not happen after fixing _get_move_to_goal
                    pass 
            
            self.current_pos = next_pos
            self.path_taken.append(self.current_pos)
            return False 
        
        return True 

# --- Setup for Simulation ---
# Re-compute paths needed for this task's examples
all_paths = compute_all_shortest_paths()

# Case 1: Successful Circumnavigation (Obstacle at 22 on path 1 -> 40) 
print("\n=== Bug2 Case 1: Successful Circumnavigation (Obstacle at 22) ===")
START_1, GOAL_1 = 1, 40
UNEXPECTED_OBSTACLE_1 = {22} # Put an obstacle at a known point on the path [cite: 157]
FULL_OBSTACLES_1 = STATIC_OBSTACLES.union(UNEXPECTED_OBSTACLE_1)
PATH_1 = all_paths.get(START_1, {}).get(GOAL_1) 
# Precomputed path: [1, 2, 3, 4, 10, 16, 22, 28, 34, 40]

if PATH_1:
    robot_1 = Bug2Robot(START_1, GOAL_1, PATH_1, FULL_OBSTACLES_1)
    
    steps = 0
    max_steps = 100
    while not robot_1.step() and steps < max_steps:
        # Prevent infinite loop by manually breaking if oscillation is detected
        if steps > 10 and len(robot_1.path_taken) > 10 and robot_1.path_taken[-1] == robot_1.path_taken[-3]:
            print("Detected oscillation. Stopping.")
            break
        steps += 1

    print(f"\nSimulation Result (Case 1): {'Success' if robot_1.current_pos == GOAL_1 else 'Failure'}")
    print(f"Final Path Taken: {robot_1.path_taken}")
else:
    print(f"No precomputed path for {START_1} to {GOAL_1}")


# Case 2: Blocked Path (Obstacle at 39, surrounded) 
print("\n=== Bug2 Case 2: Blocked Path (Obstacle at 39, surrounded) ===")
START_2, GOAL_2 = 36, 42
UNEXPECTED_OBSTACLE_2 = {39, 40} # Blocking the narrow path to 42
FULL_OBSTACLES_2 = STATIC_OBSTACLES.union(UNEXPECTED_OBSTACLE_2)
PATH_2 = all_paths.get(START_2, {}).get(GOAL_2)

if PATH_2:
    robot_2 = Bug2Robot(START_2, GOAL_2, PATH_2, FULL_OBSTACLES_2)
    
    steps = 0
    max_steps = 100
    while not robot_2.step() and steps < max_steps:
        steps += 1

    print(f"\nSimulation Result (Case 2): {'Success' if robot_2.current_pos == GOAL_2 else 'Failure'}")
    print(f"Final Path Taken: {robot_2.path_taken}")
else:
    print(f"No precomputed path for {START_2} to {GOAL_2}")