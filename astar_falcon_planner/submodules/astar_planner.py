import math
import heapq
import numpy as np
from collections import defaultdict

def plan_path(self,start, end, robot_radius, clearance, delta_time, goal_threshold, 
              wheel_radius, wheel_distance, rpm1, rpm2):

    print(f"Planning path from {start} to {end}")
    print(f"Robot parameters: radius={robot_radius}cm, clearance={clearance}cm")
    print(f"Wheel radius={wheel_radius}cm, distance={wheel_distance}cm")
    print(f"RPMs: {rpm1}, {rpm2}")
    
    # Extract positions
    sx, sy, st_deg = start
    if len(end) == 2:
        gx, gy = end
        gt_deg = 0  # Default goal orientation if not specified
    else:
        gx, gy, gt_deg = end
    
    # Convert degrees to radians
    st_rad = math.radians(st_deg)
    
    # Define action set (RPM combinations)
    actions = [
        (0, rpm1),  # Turn right
        (rpm1, 0),  # Turn left
        (rpm1, rpm1),  # Move forward
        (0, rpm2),  # Turn right faster
        (rpm2, 0),  # Turn left faster
        (rpm2, rpm2),  # Move forward faster
        (rpm1, rpm2),  # Curve right
        (rpm2, rpm1),  # Curve left
    ]
    
    # Safety check for start and goal
    total_safety_radius = robot_radius + clearance
    print(f"Total safety radius: {total_safety_radius}cm")
    if is_position_in_obstacle(sx, sy, total_safety_radius):
        self.get_logger().info("WARNING: Start position is too close to obstacles!")
        print("WARNING: Start position is too close to obstacles!")
        return           
    
    if is_position_in_obstacle(gx, gy, total_safety_radius):
        self.get_logger().info("WARNING: Goal position is too close to obstacles!")
        print("WARNING: Goal position is too close to obstacles!")
        return 
    
    # A* algorithm data structures
    open_set = []  # Priority queue
    closed_set = set()  # Set of visited nodes
    g_score = {}  # Cost from start to node
    parent = {}  # Parent node references
    path_info = {}  # Store path and action information
    
    # Initialize start node
    start_node_id = discretize_state(sx, sy, st_rad)
    g_score[start_node_id] = 0
    
    # Priority queue format: (f_score, node_id, state)
    f_score = g_score[start_node_id] + heuristic(sx, sy, gx, gy)
    heapq.heappush(open_set, (f_score, start_node_id, (sx, sy, st_rad)))
    
    # Run A* search
    iterations = 0
    max_iterations = 1000000  # Limit search to prevent infinite loops
    # no need for max iterations
    
    while open_set and iterations < max_iterations:
        iterations += 1
        
        # Get node with lowest f_score
        _, current_id, current_state = heapq.heappop(open_set)
        
        # Skip if already visited
        if current_id in closed_set:
            continue
        
        # Mark as visited
        closed_set.add(current_id)
        
        # Extract current state
        cx, cy, ctheta = current_state
        
        # Check if goal reached
        if math.hypot(cx - gx, cy - gy) <= goal_threshold:
            print(f"Goal reached after {iterations} iterations!")
            return reconstruct_path(self, parent, path_info, current_state, start_state=(sx, sy, st_rad))
        
        # Try all actions from current state
        for action in actions:
            # Simulate motion for this action
            next_state, motion_path, motion_cost = simulate_differential_drive(
                current_state, action, wheel_radius, wheel_distance, delta_time, total_safety_radius
            )
            
            # Skip if collision or invalid motion
            if next_state is None:
                continue
            
            # Extract next state
            nx, ny, ntheta = next_state
            next_id = discretize_state(nx, ny, ntheta)
            
            # Calculate tentative g_score
            tentative_g = g_score.get(current_id, float('inf')) + motion_cost
            
            # If this is a better path to next_state
            if next_id not in g_score or tentative_g < g_score[next_id]:
                # Update path information
                parent[next_id] = current_id
                g_score[next_id] = tentative_g
                path_info[next_id] = (next_state, motion_path, action)
                
                # Calculate f_score and add to open set
                f = tentative_g + heuristic(nx, ny, gx, gy)
                heapq.heappush(open_set, (f, next_id, next_state))
    
    print(f"A* search ended after {iterations} iterations without finding a path")
    
    # If no path found, return a fallback direct path
    # return simple_fallback_path(start, (gx, gy))

def discretize_state(x, y, theta):

    # Discretize position (5cm resolution)
    x_grid = round(x / 5.0)
    y_grid = round(y / 5.0)
    
    # Discretize orientation (15 degree resolution)
    # theta_grid = round(theta / (math.pi/12)) % 24
    theta_grid = round(theta / (math.pi/12)) % 24

    return (x_grid, y_grid, theta_grid)

def heuristic(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def is_position_in_obstacle(x, y, safety_radius):

    # Check map boundaries (300-600 in x, 700-1250 in y)
    if x < 1600 + safety_radius or x > 2140 - safety_radius or y < 550 + safety_radius or y > 850 - safety_radius:
        return True
    
    # Check vertical walls
    wall_thickness = 10  # cm
    
    # First wall at x=375cm
    if abs(x - 1700) < wall_thickness + safety_radius and y > 630:
        return True
    
    # Second wall at x=450cm
    if abs(x - 1810) < wall_thickness + safety_radius and y < 750:
        return True
    
    # Third wall at x=525cm
    if abs(x - 1920) < wall_thickness + safety_radius and (y < 690 or y > 750):
        return True
    
    # Fourth wall at x=575cm
    if abs(x - 2030) < wall_thickness + safety_radius and y > 630:
        return True
    
    return False

def simulate_differential_drive(state, action, wheel_radius, wheel_distance, delta_time, safety_radius):

    x, y, theta = state
    left_rpm, right_rpm = action
    
    # Convert RPM to rad/s
    left_wheel_vel = left_rpm * 2 * math.pi / 60
    right_wheel_vel = right_rpm * 2 * math.pi / 60
    
    # Simulation parameters
    steps = 10  # Number of integration steps
    dt = delta_time / steps  # Time step
    
    # Initialize path and cost
    path = [(x, y)]
    cost = 0
    
    # Record initial position for computing motion vector
    start_x, start_y, start_theta = x, y, theta
    
    # Simulate motion
    for _ in range(steps):
        # Convert wheel velocities to robot velocities
        left_vel = wheel_radius * left_wheel_vel
        right_vel = wheel_radius * right_wheel_vel
        
        # Compute robot linear and angular velocities
        v = (left_vel + right_vel) / 2  # Linear velocity
        w = (right_vel - left_vel) / wheel_distance  # Angular velocity
        
        # Save previous position for distance calculation
        prev_x, prev_y = x, y
        
        # Update state
        theta += w * dt
        theta = normalize_angle(theta)  # Keep theta in range
        
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        
        # Check for collision
        if is_position_in_obstacle(x, y, safety_radius):
            return None, None, float('inf')
        
        # Add point to path
        path.append((x, y))
        
        # Update cost
        segment_dist = math.hypot(x - prev_x, y - prev_y)
        cost += segment_dist
    
    # Compute relative motion for return
    dx = x - start_x
    dy = y - start_y
    dtheta =normalize_angle(theta - start_theta)  # Convert to degrees
    
    # Return final state, simplified motion vector, and path details
    return (x, y, theta), [(dx, dy, dtheta)], cost

def normalize_angle(theta):
    """Normalize angle to range [-π, π]"""
    return math.atan2(math.sin(theta), math.cos(theta))

def reconstruct_path(self,parent, path_info, goal_state, start_state):

    motion_commands = []
    current_state = goal_state
    current_id = discretize_state(*current_state)
    
    while current_id in parent:
        # Get path information for this step
        _, motion_path, _ = path_info[current_id]
        
        # Add motion commands to the beginning of the list
        for motion in motion_path:
            motion_commands.insert(0, list(motion))  # Convert tuple to list
        
        # Move to parent
        current_id = parent[current_id]
    self.get_logger().info(f"Reconstructed path: {motion_commands}")
    return motion_commands

