import heapq
import tkinter as tk

# Ask the user to input the grid size
GRID_SIZE = int(input("Enter the grid size: "))

# Ask the user to choose the adjacency mode
ADJACENCY_MODE = int(input("Enter the adjacency mode (4 or 8): "))

# Ask the user to input the colors for each drone
DRONE_COLORS = input("Enter the colors for each drone (separated by commas): ").split(",")

# Define the heuristic function for A* search
def heuristic(node1, node2):
    dx = abs(node1[0] - node2[0])
    dy = abs(node1[1] - node2[1])
    if ADJACENCY_MODE == 8:
        return min(dx, dy) * 14 + abs(dx - dy) * 10
    else:
        return dx + dy

# Define the function to get the neighbors of a node
def get_neighbors(node):
    x, y = node
    neighbors = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            if ADJACENCY_MODE == 4 and abs(dx) == abs(dy):
                continue
            if x + dx < 0 or x + dx >= GRID_SIZE or y + dy < 0 or y + dy >= GRID_SIZE:
                continue
            neighbors.append((x + dx, y + dy))
    return neighbors

# Define the function to perform A* search
def a_star_search(start, goal, obstacles):
    frontier = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            break
        for neighbor in get_neighbors(current):
            if neighbor in obstacles:
                continue
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(goal, neighbor)
                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path, cost_so_far[goal]

# Define the function to simulate the movement of the drones
def simulate_drones(drones):
    # Initialize the grid
    grid = [[None for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    # Initialize the time
    time = 0
    # Create the GUI window
    root = tk.Tk()
    root.title("Drone Simulation")
    # Create the canvas for drawing the grid
    canvas = tk.Canvas(root, width=GRID_SIZE*20, height=GRID_SIZE*20)
    canvas.pack()
    # Draw the grid lines
    for i in range(GRID_SIZE):
        canvas.create_line(i*20, 0, i*20, GRID_SIZE*20, fill="black")
        canvas.create_line(0, i*20, GRID_SIZE*20, i*20, fill="black")
     # Draw the drones on the grid
    for i, drone in enumerate(drones):
        x1, y1, x2, y2, start_time = drone
        color =DRONE_COLORS[i % len(DRONE_COLORS)]
        rect = canvas.create_rectangle(x1*20, y1*20, x2*20, y2*20, fill=color)
        grid[x1][y1] = rect
    # Run the simulation loop
    while True:
        # Move the drones
        for i, drone in enumerate(drones):
            x1, y1, x2, y2, start_time = drone
            if time >= start_time:
                # Erase the drone's current position on the grid
                canvas.delete(grid[x1][y1])
                grid[x1][y1] = None
                # Move the drone to its next position
                if x1 == x2:
                    # Move vertically
                    if y1 < y2:
                        y1 += 1
                    else:
                        y1 -= 1
                else:
                    # Move horizontally
                    if x1 < x2:
                        x1 += 1
                    else:
                        x1 -= 1
                # Draw the drone at its new position on the grid
                color = DRONE_COLORS[i % len(DRONE_COLORS)]
                rect = canvas.create_rectangle(x1*20, y1*20, x2*20, y2*20, fill=color)
                grid[x1][y1] = rect
                drones[i] = (x1, y1, x2, y2, start_time)
        # Update the time and sleep
        time += 1
        root.update()
        root.after(50)
    root.mainloop()


