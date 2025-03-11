import math
import heapq
import time
import pygame
import sys

# -------------------------------------------------------------------------
# Core Algorithm Functions
# -------------------------------------------------------------------------

def sample_average_refractive(i, j, gi, gj, refractive):
    """
    Sample the refractive grid along a straight line between (i, j) and (gi, gj)
    to compute an average refractive index.

    Args:
        i, j: Coordinates of the current cell.
        gi, gj: Coordinates of the goal cell.
        refractive: 2D list of refractive indices for the grid.

    Returns:
        The average refractive index along the straight line from (i,j) to (gi, gj).
    """
    samples = 5  # Use fewer samples for speed; adjust as needed.
    sum_indices = 0.0
    for k in range(samples):
        t = k / (samples - 1)
        x = i + int(t * (gi - i))
        y = j + int(t * (gj - j))
        sum_indices += refractive[x][y]
    return sum_indices / samples

def update_refractive_indices(grid, refractive, current_time):
    """
    Dynamically update refractive indices based on the current time.
    Walls ('#') are not updated.

    Args:
        grid: 2D list of characters representing the environment.
        refractive: 2D list of refractive indices for the grid.
        current_time: Current time (float), used for dynamic modulation.
    """
    n = len(grid)
    m = len(grid[0])
    for i in range(n):
        for j in range(m):
            if grid[i][j] != '#':  # Only update free space
                # Base refractive index is 1.0; modulate with a sine function.
                refractive[i][j] = 1.0 + 0.5 * math.sin(current_time + i * 0.1 + j * 0.1)

def segment_cost(px, py, qx, qy, refractive):
    """
    Compute the cost to travel from point (px, py) to (qx, qy) by sampling
    the refractive index along the straight line connecting them.

    Args:
        px, py: Coordinates of the start point.
        qx, qy: Coordinates of the end point.
        refractive: 2D list of refractive indices.

    Returns:
        The approximated cost as: distance * (average refractive index).
    """
    dist = math.hypot(qx - px, qy - py)
    samples = 5
    sum_indices = 0.0
    for k in range(samples):
        t = k / (samples - 1)
        x = int(px + t * (qx - px))
        y = int(py + t * (qy - py))
        sum_indices += refractive[x][y]
    avg_index = sum_indices / samples
    return dist * avg_index

def refine_path(discrete_path, refractive, iterations=20, learning_rate=0.1):
    """
    Refine a discrete path (given as a list of grid coordinates) into a smooth,
    continuous path using a gradient-descent-like smoothing procedure.

    Args:
        discrete_path: List of tuples representing grid coordinates of the path.
        refractive: 2D list of refractive indices.
        iterations: Number of smoothing iterations.
        learning_rate: Step size for updating each point.

    Returns:
        A list of tuples (floats) representing the refined continuous path.
    """
    # Initialise continuous path as float coordinates.
    path = [(float(pt[0]), float(pt[1])) for pt in discrete_path]

    for _ in range(iterations):
        # Iterate over all intermediate points (excluding start and goal).
        for i in range(1, len(path) - 1):
            # Calculate the current cost for segments before and after the point.
            current_cost = (segment_cost(path[i-1][0], path[i-1][1],
                                         path[i][0],   path[i][1],   refractive) +
                            segment_cost(path[i][0],   path[i][1],
                                         path[i+1][0], path[i+1][1], refractive))
            best_dx = 0
            best_dy = 0
            best_cost = current_cost

            # Try small perturbations in all directions.
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    new_x = path[i][0] + dx * learning_rate
                    new_y = path[i][1] + dy * learning_rate
                    new_cost = (segment_cost(path[i-1][0], path[i-1][1],
                                             new_x,        new_y,        refractive) +
                                segment_cost(new_x,        new_y,
                                             path[i+1][0], path[i+1][1], refractive))
                    if new_cost < best_cost:
                        best_cost = new_cost
                        best_dx = dx * learning_rate
                        best_dy = dy * learning_rate
            # Update point with the perturbation that reduces cost the most.
            path[i] = (path[i][0] + best_dx, path[i][1] + best_dy)
    return path

def run_pathfinding(grid, refractive, start, goal):
    """
    Run the A* search algorithm on the grid to find the minimum-cost path from
    the start to the goal. The cost of moving through a cell is determined by its
    refractive index.

    Args:
        grid: 2D list of characters representing the environment.
        refractive: 2D list of refractive indices.
        start: Tuple (i, j) representing the start cell.
        goal: Tuple (i, j) representing the goal cell.

    Returns:
        A list of tuples representing the discrete path from start to goal.
    """
    n = len(grid)
    m = len(grid[0])
    # Initialise cost and parent arrays.
    dist = [[math.inf for _ in range(m)] for _ in range(n)]
    parent = [[(-1, -1) for _ in range(m)] for _ in range(n)]

    # Heuristic function: Euclidean distance multiplied by average refractive index.
    def heuristic(i, j):
        dx = i - goal[0]
        dy = j - goal[1]
        distance = math.sqrt(dx * dx + dy * dy)
        avg_index = sample_average_refractive(i, j, goal[0], goal[1], refractive)
        return distance * avg_index

    pq = []  # Priority queue (min-heap)
    dist[start[0]][start[1]] = 0
    heapq.heappush(pq, (heuristic(start[0], start[1]), start[0], start[1], 0.0))
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    # A* search loop.
    while pq:
        priority, cx, cy, cost_so_far = heapq.heappop(pq)
        if (cx, cy) == goal:
            break
        if cost_so_far > dist[cx][cy]:
            continue
        for dx, dy in directions:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or ny < 0 or nx >= n or ny >= m or grid[nx][ny] == '#':
                continue
            new_cost = cost_so_far + refractive[nx][ny]
            if new_cost < dist[nx][ny]:
                dist[nx][ny] = new_cost
                parent[nx][ny] = (cx, cy)
                heapq.heappush(pq, (new_cost + heuristic(nx, ny), nx, ny, new_cost))
    
    # Reconstruct the discrete path by backtracking from goal to start.
    discrete_path = []
    cur = goal
    if math.isinf(dist[goal[0]][goal[1]]):
        return []  # No path found.
    while cur != start:
        discrete_path.append(cur)
        cur = parent[cur[0]][cur[1]]
    discrete_path.append(start)
    discrete_path.reverse()
    return discrete_path

# -------------------------------------------------------------------------
# Pygame Visualisation with Mouse Interaction
# -------------------------------------------------------------------------

def main():
    # Define the initial grid as a list of strings.
    original_grid = [
        "#########################",
        "#S       #         ##   #",
        "#   ######  #####  ##   #",
        "#       ##      #####   #",
        "#   ########       ##   #",
        "#          ####         #",
        "#   ######        #######",
        "#       ##          G   #",
        "#       #######         #",
        "#########################"
    ]
    n = len(original_grid)
    m = len(original_grid[0])

    # Convert the grid into a mutable list of lists.
    grid = [list(row) for row in original_grid]

    # Create a refractive grid with initial values.
    refractive = [[1.0 for _ in range(m)] for _ in range(n)]
    for i in range(3, min(6, n)):
        for j in range(10, min(18, m)):
            if grid[i][j] == ' ':
                refractive[i][j] = 1.5

    # Locate the start (S) and goal (G) positions.
    start = None
    goal = None
    for i in range(n):
        for j in range(m):
            if grid[i][j] == 'S':
                start = (i, j)
            elif grid[i][j] == 'G':
                goal = (i, j)
    if not start or not goal:
        print("Start or Goal not found in the grid!")
        sys.exit(1)

    # Set up the Pygame window.
    pygame.init()
    cell_size = 30
    window_width = m * cell_size
    window_height = n * cell_size
    screen = pygame.display.set_mode((window_width, window_height))
    pygame.display.set_caption("Optics-Inspired Pathfinding")
    clock = pygame.time.Clock()

    running = True
    while running:
        # Event handling.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            # Allow left click to add a wall and right click to remove a wall.
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_x, mouse_y = event.pos
                grid_i = mouse_y // cell_size
                grid_j = mouse_x // cell_size
                # Ensure the click is within bounds.
                if 0 <= grid_i < n and 0 <= grid_j < m:
                    # Left click adds a wall.
                    if event.button == 1:
                        grid[grid_i][grid_j] = '#'
                    # Right click removes a wall.
                    elif event.button == 3:
                        grid[grid_i][grid_j] = ' '

        # Update dynamic refractive indices.
        current_time = time.perf_counter()
        update_refractive_indices(grid, refractive, current_time)

        # Compute the path using A* search.
        discrete_path = run_pathfinding(grid, refractive, start, goal)
        # Optionally compute the refined continuous path.
        refined_path = refine_path(discrete_path, refractive) if discrete_path else []

        # Clear the screen.
        screen.fill((0, 0, 0))

        # Draw the grid.
        for i in range(n):
            for j in range(m):
                rect = pygame.Rect(j * cell_size, i * cell_size, cell_size, cell_size)
                if grid[i][j] == '#':
                    color = (100, 100, 100)
                else:
                    # Map the refractive index to a color (blue for low, red for high).
                    r_index = refractive[i][j]
                    red = max(0, min(255, int((r_index - 1.0) * 255)))
                    blue = 255 - red
                    color = (red, 50, blue)
                pygame.draw.rect(screen, color, rect)
                # Optionally draw grid lines.
                pygame.draw.rect(screen, (0, 0, 0), rect, 1)

        # Draw start (green) and goal (red) cells.
        start_rect = pygame.Rect(start[1] * cell_size, start[0] * cell_size, cell_size, cell_size)
        pygame.draw.rect(screen, (0, 255, 0), start_rect)
        goal_rect = pygame.Rect(goal[1] * cell_size, goal[0] * cell_size, cell_size, cell_size)
        pygame.draw.rect(screen, (255, 0, 0), goal_rect)

        # Draw the discrete path if found (yellow line).
        if discrete_path:
            points = [(y * cell_size + cell_size // 2, x * cell_size + cell_size // 2) for (x, y) in discrete_path]
            if len(points) > 1:
                pygame.draw.lines(screen, (255, 255, 0), False, points, 3)

        # Optionally draw the refined continuous path as a smoother line (cyan).
        if refined_path:
            smooth_points = [(int(pt[1] * cell_size + cell_size / 2), int(pt[0] * cell_size + cell_size / 2)) for pt in refined_path]
            if len(smooth_points) > 1:
                pygame.draw.lines(screen, (0, 255, 255), False, smooth_points, 2)

        pygame.display.flip()
        clock.tick(30)  # Limit to ~30 FPS

    pygame.quit()

if __name__ == "__main__":
    main()
