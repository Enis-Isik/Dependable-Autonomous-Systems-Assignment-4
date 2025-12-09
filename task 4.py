# Import all utilities
from warehouse_utils import *

# Define a safe holding spot adjacent to the conflict path, accessible in ONE step (Right-angle move).
HOLDING_SPOT = 23 # Position 23 is adjacent (Right) to 22.

class CoordinatedRobot:
    def __init__(self, robot_id, start_pos, goal_pos, path_plan):
        self.id = robot_id 
        self.current_pos = start_pos
        self.goal_pos = goal_pos
        self.path_plan = path_plan
        self.path_taken = [start_pos]
        
        try:
            self.path_index = path_plan.index(start_pos)
        except ValueError:
            self.path_index = 0
            
        self.state = "MOVE" # MOVE, HOLDING, RESUMING_PATH
        self.conflict_pos = None

    def get_next_on_plan(self):
        """Gets the next position on the planned route based on path_index."""
        if self.path_index + 1 < len(self.path_plan):
            return self.path_plan[self.path_index + 1]
        return None
    
    def get_current_target(self):
        """Returns the intended next move based on state (for logging/predictability)."""
        if self.state == "MOVE":
            return self.get_next_on_plan()
        elif self.state == "HOLDING":
            return HOLDING_SPOT
        elif self.state == "RESUMING_PATH":
            # Target is the spot on the main path, which is where the conflict occurred (28)
            return self.path_plan[self.path_index + 1]
        return None

    def check_for_conflict(self, predicted_next_pos, other_robot):
        """Checks for predicted collisions in the next step."""
        other_next_move = other_robot.get_current_target()
        
        # Check if the other robot is mid-maneuver (HOLDING/RESUMING) and adjust prediction if needed
        if other_robot.state != "MOVE" and other_robot.id == 1:
            # If R1 is mid-maneuver, R2 assumes R1 will move to the holding spot or resume spot
            other_next_move = other_robot.get_current_target() 
        
        # Head-to-Head
        if predicted_next_pos == other_next_move:
            return True, predicted_next_pos
            
        # Swapping
        if predicted_next_pos == other_robot.current_pos and self.current_pos == other_next_move:
            return True, predicted_next_pos
            
        return False, None

    def step(self, other_robot):
        if self.current_pos == self.goal_pos:
            return True

        next_pos = self.current_pos # Default is to hold position

        # --- 1. State Machine Execution ---
        if self.state == "MOVE":
            next_on_plan = self.get_next_on_plan()
            if next_on_plan is None: return True
            
            is_conflict, conflict_pos = self.check_for_conflict(next_on_plan, other_robot)
            
            if is_conflict and self.id == 1: # R1 (Lower ID) must yield
                self.conflict_pos = conflict_pos
                self.state = "HOLDING"
                # R1's move in this step is the side-step action (e.g., 22 -> 23)
                next_pos = HOLDING_SPOT 
                print(f"Robot {self.id}: SWAP/HEAD_TO_HEAD detected at {conflict_pos}. Initiating Side-Step to {HOLDING_SPOT}.")
            elif is_conflict and self.id == 2: # R2 (Higher ID) proceeds
                next_pos = next_on_plan
                self.path_index += 1
            else: # Normal move
                next_pos = next_on_plan
                self.path_index += 1

        elif self.state == "HOLDING":
            # R1 is at HOLDING_SPOT (e.g., 23). Check if R2 has cleared the path segment.
            
            # The critical segment is R1's next spot (conflict_pos) and the spot after that (R2's next spot).
            # We assume R2 has cleared the path if its current index is beyond the conflict spot index.
            conflict_index = self.path_plan.index(self.conflict_pos)
            
            # Note: R2's path index advances faster because it is the priority robot.
            if other_robot.path_index >= conflict_index:
                # R2 has successfully moved to the conflict spot or beyond it. Time to resume.
                self.state = "RESUMING_PATH"
                # R1 moves back onto the path: HOLDING_SPOT -> Path position (e.g., 22)
                next_pos = self.path_plan[self.path_index] 
                print(f"Robot {self.id}: Conflict clear (R2 at index {other_robot.path_index}). Resuming to path position {next_pos}.")
            else:
                next_pos = self.current_pos # Hold at HOLDING_SPOT

        elif self.state == "RESUMING_PATH":
            # R1 moves forward from the path spot it just landed on (e.g., 22 -> 28)
            
            # The move in the previous step was to get *onto* the path spot (22). 
            # Now we must officially advance the path index and move to the next planned spot (28).
            self.path_index += 1
            next_pos = self.get_next_on_plan()
            self.state = "MOVE"
            print(f"Robot {self.id}: Resumed path at {self.current_pos}. Moving to {next_pos}.")

        # --- 2. Execute Move ---
        self.current_pos = next_pos
        self.path_taken.append(self.current_pos)
        return False
    
# --- Setup for Simulation ---
# Re-compute paths needed for this task's examples
all_paths = compute_all_shortest_paths() 

START_1, GOAL_1 = 4, 41
START_2, GOAL_2 = 41, 4

PATH_R1 = all_paths.get(START_1, {}).get(GOAL_1)
PATH_R2 = all_paths.get(START_2, {}).get(GOAL_2) 

if not PATH_R1 or not PATH_R2:
    print("Error: Could not retrieve shortest paths from utilities.")
    exit()

# Start R1 at 4 and R2 at 41 to match path plan starting points
robot_1 = CoordinatedRobot(1, PATH_R1[0], GOAL_1, PATH_R1)
robot_2 = CoordinatedRobot(2, PATH_R2[0], GOAL_2, PATH_R2)

print("--- Dynamic Collision Avoidance Simulation (FINAL COMPLIANT FIX) ---")
print(f"R1 Path: {robot_1.path_plan}")
print(f"R2 Path: {robot_2.path_plan}")

steps = 0
max_steps = 25
while (robot_1.current_pos != GOAL_1 or robot_2.current_pos != GOAL_2) and steps < max_steps:
    steps += 1
    
    # R1 decides its action
    r1_finished = robot_1.step(robot_2)
    # R2 decides its action
    r2_finished = robot_2.step(robot_1) 
    
    print(f"\nTime Step {steps}:")
    print(f"R1 (ID {robot_1.id}): Pos {robot_1.current_pos}, State {robot_1.state}, Target: {robot_1.get_current_target()}, Index: {robot_1.path_index}")
    print(f"R2 (ID {robot_2.id}): Pos {robot_2.current_pos}, State {robot_2.state}, Target: {robot_2.get_current_target()}, Index: {robot_2.path_index}")

print("\n--- Simulation Complete ---")

# Final Safety Check
print(f"Robot 1 Final Path Taken: {robot_1.path_taken}")
print(f"Robot 2 Final Path Taken: {robot_2.path_taken}")
collision_points = set()
for t in range(min(len(robot_1.path_taken), len(robot_2.path_taken))):
    if robot_1.path_taken[t] == robot_2.path_taken[t]:
        collision_points.add((robot_1.path_taken[t], t))
        
print(f"\nFinal Safety Check: Collisions Detected: {collision_points} (Empty set means safety guaranteed)")