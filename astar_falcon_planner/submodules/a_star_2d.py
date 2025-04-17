import numpy as np
import math
import matplotlib.pyplot as plt
import heapq
import cv2
from matplotlib import colors
import time

# Define the colors: 0 = Free (white), 1 = Obstacle (black), 2 = Clearance (orange)
cmap = colors.ListedColormap(['white', 'black', 'orange'])
bounds = [0, 0.5, 1.5, 2.5]
norm = colors.BoundaryNorm(bounds, cmap.N)
clear=int(input("Enter robot clearance (in mm) = "))
s1=int(input("Enter first angular velocity = "))
s2=int(input("Enter second angular velocity = "))

scale = 200
h=0
clear_process=clear*scale/1000
actual_width = 5.4
actual_height= 3
canvas_width = int(actual_width*scale)
canvas_height = int(actual_height*scale)
robot_radius_real=0.22
robot_radius=robot_radius_real*scale
c=int(robot_radius+clear_process)

XY_RESOLUTION = 0.1*scale
THETA_RES = 30
NUM_THETA = 360 // THETA_RES

def get_start_goal_inputs(obstacle_mask, clearance_mask):
    """Get start and goal node inputs from user with validation."""
    def get_input(node_type):
        while True:
            try:
                # print(f"\n===== {node_type.upper()} NODE =====")
                x_real = float(input(f"Enter {node_type} X (in meters, 0 - {actual_width}): "))
                y_real = float(input(f"Enter {node_type} Y (in meters, 0 - {actual_height}): "))
                theta = float(input(f"Enter {node_type} Theta (multiple of 30, 0-360): "))

                # Convert to pixel coordinates
                x = int(x_real * scale)
                y = int(y_real * scale)

                if not (0 <= x < canvas_width and 0 <= y < canvas_height):
                    print("Coordinates out of canvas bounds.")
                    continue

                if obstacle_mask[y, x] == 1:
                    print("Location is inside an obstacle.")
                    continue

                if clearance_mask[y, x] == 2:
                    print("Location is in a clearance zone.")
                    continue

                if theta % 30 != 0 or not (0 <= theta <= 360):
                    print("Theta must be a multiple of 30 between 0 and 360.")
                    continue

                theta = theta % 360
                print(f"{node_type.capitalize()} position ({x_real:.2f}m, {y_real:.2f}m, {theta:.1f}°) is valid.")
                return (x, y, theta)

            except ValueError:
                print("Invalid input. Please enter numbers only.")

    # print("\n===== PATH PLANNING CONFIGURATION =====")
    print(f"Canvas size: {canvas_width} x {canvas_height} pixels ({actual_width}m x {actual_height}m)")
    print("Enter coordinates in meters. Recommended to stay away from borders for clearance.\n")

    start = get_input("start")
    goal = get_input("goal") if start else None
    return start, goal



def point_inside_obstacle(x, y):
    scale = 200
    if x < 0.1*scale or x > canvas_width - 0.1*scale or y < scale*0.1 or y > canvas_height - 0.1*scale:
        return True
    if (scale*1.0 <= x <= scale*(1.0 + 0.1)) and (0 <= y <= scale*(3.0 - 0.6)):
        return True
    if (scale*2.1 <= x <= scale*(2.1 + 0.1)) and (scale*0.6 <= y <= scale*3.0):
        return True
    if (scale*3.2 <= x <= scale*(3.2 + 0.1)):
        if (0 <= y <= scale*1.25) or (scale*(3.0 - 1.25) <= y <= scale*3.0):
            return True
    if (scale*4.3 <= x <= scale*(4.3 + 0.1)) and (0 <= y <= scale*(3.0 - 0.6)):
        return True
    return False

def create_obstacles():
    map_img = 255 * np.ones((canvas_height, canvas_width, 3), dtype=np.uint8)
    obstacle_map = np.zeros((canvas_height, canvas_width), dtype=np.uint8)
    for y in range(canvas_height):
        for x in range(canvas_width):
            if point_inside_obstacle(x, y):
                obstacle_map[y, x] = 1
                map_img[y, x] = (0, 0, 0)
    return map_img, obstacle_map

def clearance_obstacles(clearance):
    print("clearance = ", clearance)
    _, obstacle_mask = create_obstacles()
    kernel = np.ones((2*clearance, 2*clearance), np.uint8)
    clearance_mask = cv2.dilate(obstacle_mask, kernel, iterations=1)
    clearance_mask[(clearance_mask == 1) & (obstacle_mask == 0)] = 2
    clearance_mask[obstacle_mask == 1] = 1
    return clearance_mask

def cost(Xi, Yi, Thetai, UL, UR, clearance_map):
    t = 0
    r = 0.0335*scale
    L = 0.16*scale
    dt = 0.1
    Xn = Xi
    Yn = Yi
    Thetan = math.radians(Thetai)
    D = 0
    path_x, path_y = [], []

    while t < 1:
        t += dt
        Delta_Xn = 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
        Delta_Yn = 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
        Xn += Delta_Xn
        Yn += Delta_Yn
        Thetan += (r / L) * (UR - UL) * dt
        D += math.hypot(Delta_Xn, Delta_Yn)
        path_x.append(Xn)
        path_y.append(Yn)

        x_map = int(Xn)
        y_map = int(Yn)
        if (x_map < 0 or x_map >= canvas_width or y_map < 0 or y_map >= canvas_height or
            clearance_map[y_map, x_map] == 1 or clearance_map[y_map, x_map] == 2):
            return 3

    Thetan = math.degrees(Thetan) % 360
    return Xn, Yn, Thetan, D, path_x, path_y

def heuristic(x, y, goal):
    return math.hypot(goal[0] - x, goal[1] - y)

class Node:
    def __init__(self, x, y, theta, g, h, parent=None, path_x=None, path_y=None, action=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        self.path_x = path_x or []
        self.path_y = path_y or []
        self.action = action

    def __lt__(self, other):
        return self.f < other.f

def state_to_index(x, y, theta):
    x_idx = int(x / XY_RESOLUTION)
    y_idx = int(y / XY_RESOLUTION)
    theta_idx = int(theta // THETA_RES) % NUM_THETA
    return x_idx, y_idx, theta_idx

def backtrack_path(node, ax):
    path_x = []
    path_y = []
    actions_taken = []

    while node.parent:
        ax.plot(node.path_x, node.path_y, color='green', linewidth=2)
        path_x = node.path_x + path_x
        path_y = node.path_y + path_y
        actions_taken.insert(0, node.action)
        node = node.parent


    with open("final_path_actions.txt", "w") as f:
        f.write("Step\tUL\tUR\n")
        for i, act in enumerate(actions_taken):
            f.write(f"{i+1}\t{act[0]}\t{act[1]}\n")

    return path_x, path_y

def astar_with_matrix(start, goal, actions):
    clearance_map = clearance_obstacles(clearance=c)
    x_size = int(canvas_width / XY_RESOLUTION)
    y_size = int(canvas_height / XY_RESOLUTION)
    V = np.zeros((x_size, y_size, NUM_THETA), dtype=bool)
    h=0
    open_list = []
    fig, ax = plt.subplots()
    ax.set_title("A* Non-Holonomic Search")
    ax.set_xlim(0, canvas_width)
    ax.set_ylim(0, canvas_height)

    ax.imshow(clearance_map, cmap=cmap, norm=norm, origin='lower', extent=[0, canvas_width, 0, canvas_height], alpha=1.0)
    goal_radius = 0.25*scale
    goal_circle = plt.Circle((goal[0], goal[1]), goal_radius, color='red', fill=False, linewidth=2)
    ax.add_patch(goal_circle)
    ax.scatter(start[0], start[1], color='red', marker='X', s=100, label="Start")
    ax.scatter(goal[0], goal[1], color='green', marker='X', s=100, label="Goal")

    start_node = Node(start[0], start[1], start[2], 0, heuristic(start[0], start[1], goal))
    heapq.heappush(open_list, start_node)
    gem = 0
    frame_count = 0
    start_time = time.time()
    while open_list:
        current = heapq.heappop(open_list)
        gem += 1

        if heuristic(current.x, current.y, goal) <= goal_radius:
            path_x, path_y = backtrack_path(current, ax)
            ax.plot(path_x, path_y, color='blue', linewidth=3, label="Final Path")            
            ax.legend(loc="upper right")
            plt.show()
            print("Goal reached")
            end_time=time.time()
            print(f"Time taken: {end_time - start_time:.2f} seconds")
            return current, ax

        x_idx, y_idx, theta_idx = state_to_index(current.x, current.y, current.theta)
        if V[x_idx, y_idx, theta_idx]:
            continue
        V[x_idx, y_idx, theta_idx] = True

        ax.plot(current.x, current.y, marker='o', color='yellow', markersize=1)  # Expanded node

        for action in actions:
            if cost(current.x, current.y, current.theta, action[0], action[1], clearance_map) == 3:
                continue

            xn, yn, thetan, cost_to_come, path_x, path_y = cost(current.x, current.y, current.theta, action[0], action[1], clearance_map)

            if not (0 <= xn < canvas_width and 0 <= yn < canvas_height):
                continue

            x_map = int(xn)
            y_map = int(yn)
            if clearance_map[y_map, x_map] == 1:
                continue

            g_new = current.g + cost_to_come
            h_new = heuristic(xn, yn, goal)

            x_idx, y_idx, theta_idx = state_to_index(xn, yn, thetan)
            found_in_open_list = False
            for node_in_open_list in open_list:
                if node_in_open_list.x == xn and node_in_open_list.y == yn and node_in_open_list.theta == thetan:
                    found_in_open_list = True
                    if node_in_open_list.g > g_new:
                        node_in_open_list.g = g_new
                        node_in_open_list.f = g_new + h_new
                        node_in_open_list.parent = current
                        node_in_open_list.path_x = path_x
                        node_in_open_list.path_y = path_y
                        node_in_open_list.action = action
                    break

            if not found_in_open_list:
                new_node = Node(xn, yn, thetan, g_new, h_new, parent=current, path_x=path_x, path_y=path_y, action=action)
                heapq.heappush(open_list, new_node)
                ax.plot(xn, yn, marker='o', color='purple', markersize=1)  # Frontier node

            ax.plot(path_x, path_y, color='gray', linewidth=0.4)

        frame_count += 1
        if 0<=goal[0] and goal[0]<400:
            if frame_count % 10 == 0:
                plt.pause(0.001)

        elif 400<=goal[0] and goal[0]<600:
            if frame_count % 50 == 0:
                plt.pause(0.001)

        elif 600<=goal[0] and goal[0]<800:
            if frame_count % 100 == 0:
                plt.pause(0.001)
        elif 800<=goal[0] and goal[0]<1000:
            if frame_count % 300 == 0:
                plt.pause(0.001)
        

    print("Path not found")
    return None, ax

if __name__ == "__main__":
    actions = [[0, s1], [s1, 0], [s1, s1], [s2, 0], [0, s2], [s2, s2], [s2, s1], [s1, s2]]

    # Step 1: Generate obstacle and clearance maps
    map_img, obstacle_mask = create_obstacles()
    clearance_mask = clearance_obstacles(c)

    # Step 2: Get valid start and goal input from the user
    start, goal = get_start_goal_inputs(obstacle_mask, clearance_mask)

    # Step 3: Run A* if inputs were valid
    if start and goal:
        result, ax = astar_with_matrix(start, goal, actions)
    else:
        print("Invalid start or goal position. Exiting.")

