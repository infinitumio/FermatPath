import random

def generate_maze(width=50, height=50):
    maze = [['#' for _ in range(width)] for _ in range(height)]
    
    # Carve a path using a simple randomized DFS algorithm
    def carve_path(x, y):
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        random.shuffle(directions)
        for dx, dy in directions:
            nx, ny = x + dx * 2, y + dy * 2
            if 1 <= nx < height - 1 and 1 <= ny < width - 1 and maze[nx][ny] == '#':
                maze[x + dx][y + dy] = ' '
                maze[nx][ny] = ' '
                carve_path(nx, ny)
    
    # Place start 'S'
    sx, sy = 1, 1
    maze[sx][sy] = 'S'
    
    # Carve the initial path
    carve_path(sx, sy)
    
    # Place goal 'G' at the farthest open space
    farthest_x, farthest_y = sx, sy
    for i in range(1, height - 1):
        for j in range(1, width - 1):
            if maze[i][j] == ' ' and (i + j) > (farthest_x + farthest_y):
                farthest_x, farthest_y = i, j
    maze[farthest_x][farthest_y] = 'G'
    
    return ['"' + ''.join(row) + '",' for row in maze]

# Generate and print the maze
maze = generate_maze(50, 50)
for row in maze:
    print(row)
