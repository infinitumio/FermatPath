#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <string>
#include <algorithm>
#include <chrono>

// -------------------------------------------------------------------------
// Node and Comparator for A* Search
// -------------------------------------------------------------------------

// Node definition: each node holds its grid coordinates, the cost from the start,
// and a priority (cost + heuristic) used in the A* search.
struct Node {
    int x, y;
    double cost;     // Accumulated cost (simulated travel time through the medium)
    double priority; // Sum of cost and heuristic (mimicking Fermat's principle)
};

// Comparator for the priority queue: nodes with lower priority (cost + heuristic)
// are given higher priority (i.e., they are expanded first).
struct CompareNode {
    bool operator()(const Node& a, const Node& b) {
        return a.priority > b.priority;
    }
};

// -------------------------------------------------------------------------
// Core Algorithm Functions
// -------------------------------------------------------------------------

// sampleAverageRefractive:
// Samples the refractive grid along a straight line between (i, j) and (gi, gj)
// and returns the average refractive index along that line.
double sampleAverageRefractive(int i, int j, int gi, int gj,
    const std::vector<std::vector<double>>& refractive) {
    double sum = 0.0;
    int samples = 5; // Use fewer samples for speed; adjust as needed.
    for (int k = 0; k < samples; k++) {
        double t = static_cast<double>(k) / (samples - 1);
        // Compute the sample point along the straight line.
        int x = i + static_cast<int>(t * (gi - i));
        int y = j + static_cast<int>(t * (gj - j));
        sum += refractive[x][y];
    }
    return sum / samples;
}

// updateRefractiveIndices:
// Dynamically updates the refractive indices based on the current time.
// Walls (cells with '#') are left unchanged.
void updateRefractiveIndices(const std::vector<std::string>& grid,
    std::vector<std::vector<double>>& refractive,
    double time) {
    int n = grid.size();
    int m = grid[0].size();
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            // Only update free space (cells that are not walls).
            if (grid[i][j] != '#') {
                // Base refractive index is 1.0; add dynamic modulation with sine function.
                refractive[i][j] = 1.0 + 0.5 * std::sin(time + i * 0.1 + j * 0.1);
            }
        }
    }
}

// segmentCost:
// Computes the cost to travel from point (px, py) to (qx, qy) by sampling
// the refractive index along the straight line between them.
// This approximates the integral of the refractive index along the path.
double segmentCost(double px, double py, double qx, double qy,
    const std::vector<std::vector<double>>& refractive) {
    // Calculate Euclidean distance between the two points.
    double dist = std::hypot(qx - px, qy - py);
    int samples = 5; // Number of samples used for numerical integration.
    double sum = 0.0;
    for (int k = 0; k < samples; k++) {
        double t = static_cast<double>(k) / (samples - 1);
        // Calculate the sample coordinates.
        int x = static_cast<int>(px + t * (qx - px));
        int y = static_cast<int>(py + t * (qy - py));
        sum += refractive[x][y];
    }
    double avgIndex = sum / samples;
    return dist * avgIndex;
}

// refinePath:
// Refines a discrete path (given as integer grid coordinates) to produce a smooth,
// continuous path using a gradient-descent�like method.
std::vector<std::pair<double, double>> refinePath(
    const std::vector<std::pair<int, int>>& discretePath,
    const std::vector<std::vector<double>>& refractive,
    int iterations = 20, double learningRate = 0.1) {

    // Initialize continuous path using the discrete path coordinates.
    std::vector<std::pair<double, double>> path;
    for (auto& pt : discretePath) {
        path.push_back({ static_cast<double>(pt.first), static_cast<double>(pt.second) });
    }

    // Iteratively refine the intermediate points (keeping start and goal fixed).
    for (int iter = 0; iter < iterations; iter++) {
        for (size_t i = 1; i < path.size() - 1; i++) {
            // Compute the current cost (optical path length) from neighbor segments.
            double currentCost = segmentCost(path[i - 1].first, path[i - 1].second,
                path[i].first, path[i].second, refractive) +
                segmentCost(path[i].first, path[i].second,
                    path[i + 1].first, path[i + 1].second, refractive);
            double bestDx = 0, bestDy = 0, bestCost = currentCost;
            // Try small perturbations around the current point.
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0)
                        continue;
                    double newX = path[i].first + dx * learningRate;
                    double newY = path[i].second + dy * learningRate;
                    double newCost = segmentCost(path[i - 1].first, path[i - 1].second,
                        newX, newY, refractive) +
                        segmentCost(newX, newY,
                            path[i + 1].first, path[i + 1].second, refractive);
                    if (newCost < bestCost) {
                        bestCost = newCost;
                        bestDx = dx * learningRate;
                        bestDy = dy * learningRate;
                    }
                }
            }
            // Update the current point if a better cost is found.
            path[i].first += bestDx;
            path[i].second += bestDy;
        }
    }
    return path;
}

// run_pathfinding:
// Implements A* search on the grid to compute a discrete path from the start to the goal.
// The cost for moving into each cell is based on its refractive index.
std::vector<std::pair<int, int>> run_pathfinding(
    const std::vector<std::string>& grid,
    const std::vector<std::vector<double>>& refractive,
    const std::pair<int, int>& start,
    const std::pair<int, int>& goal) {

    int n = grid.size();
    int m = grid[0].size();
    // Initialize the cost matrix with infinity.
    std::vector<std::vector<double>> dist(n, std::vector<double>(m, std::numeric_limits<double>::infinity()));
    // Parent pointers for path reconstruction.
    std::vector<std::vector<std::pair<int, int>>> parent(n, std::vector<std::pair<int, int>>(m, { -1, -1 }));

    // Heuristic function: Euclidean distance multiplied by an average refractive index.
    auto heuristic = [&](int i, int j) {
        double dx = i - goal.first;
        double dy = j - goal.second;
        double distance = std::sqrt(dx * dx + dy * dy);
        double avgIndex = sampleAverageRefractive(i, j, goal.first, goal.second, refractive);
        return distance * avgIndex;
        };

    // Priority queue for A* search.
    std::priority_queue<Node, std::vector<Node>, CompareNode> pq;
    dist[start.first][start.second] = 0;
    pq.push({ start.first, start.second, 0, heuristic(start.first, start.second) });
    // Define possible moves: up, down, left, right.
    std::vector<std::pair<int, int>> directions = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };

    // A* Search loop.
    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        // If the goal is reached, break out of the loop.
        if (current.x == goal.first && current.y == goal.second)
            break;

        // Skip if a better path has already been found.
        if (current.cost > dist[current.x][current.y])
            continue;

        // Check all neighbor cells.
        for (auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;

            // Check for boundaries and obstacles.
            if (nx < 0 || ny < 0 || nx >= n || ny >= m || grid[nx][ny] == '#')
                continue;

            // Compute new cost (cost to move into neighbor cell).
            double newCost = current.cost + refractive[nx][ny];
            if (newCost < dist[nx][ny]) {
                dist[nx][ny] = newCost;
                parent[nx][ny] = { current.x, current.y };
                double priority = newCost + heuristic(nx, ny);
                pq.push({ nx, ny, newCost, priority });
            }
        }
    }

    // Reconstruct the discrete path by backtracking from the goal to the start.
    std::vector<std::pair<int, int>> discretePath;
    std::pair<int, int> cur = goal;
    if (std::isinf(dist[goal.first][goal.second]))
        return {};  // No path found.
    while (cur != start) {
        discretePath.push_back(cur);
        cur = parent[cur.first][cur.second];
    }
    discretePath.push_back(start);
    std::reverse(discretePath.begin(), discretePath.end());
    return discretePath;
}

// -------------------------------------------------------------------------
// Main Function and Pygame Visualization with Mouse Interaction
// -------------------------------------------------------------------------

int main() {
    // Define the grid as a vector of strings.
    // Walls are '#' and free spaces are ' '. The start 'S' and goal 'G' are marked.
    std::vector<std::string> grid = {
"##################################################",
"#S#   #   #               #   #       #   #     ##",
"# # # # # ### ######### # # # # ### ### # # ### ##",
"#   #   #   #   #   #   #   # #   #     #   #   ##",
"########### ##### # # ### ##### ############# # ##",
"#     #     #     #   # # #     #   #   #     # ##",
"# # ### ##### ######### # # ##### # # # # # ######",
"# # #   #         # #     #     # #   # # # #   ##",
"### # ### ####### # # ######### # ##### # ### # ##",
"#   # # #   #   # # # #   #     #   #     #   # ##",
"# ### # ### # # # # # # # # ####### ####### ### ##",
"# #   # #   # #   #   # # # #     #         #   ##",
"# # ### # ### ####### ### # ### # ########### # ##",
"#   #       # #     #   # #   # #   #     #   # ##",
"# ########### # ### ### # ### # ### # ### # ### ##",
"# #       #   #   #   # # #   # #     #   # # # ##",
"# # ##### # ##### ### # # # ### ####### ### # # ##",
"# # #   # #       # # #   #     #     #   # #   ##",
"# # # ### # ####### # ######### # ### ### # ### ##",
"#   #     #         #           # # # # # #   # ##",
"##### ############# ########### # # # # # ### ####",
"#   #   #       #       #     # # # # #   # #   ##",
"# ##### ### # # # ####### ### # # # # ### # ### ##",
"#     #   # # # # # #     #   # # # #   # # #   ##",
"# ### ### # # ### # # ##### ##### # ### # # # # ##",
"# #     # # #   # # #     #       #   # #   # # ##",
"### ##### # ### # # ##### ######### ### ##### ####",
"#   #     #   # # # #   #     #         #   #   ##",
"# ### ######### # # # # ### # ######### # # ### ##",
"#   # #       #     # #   # #         #   #     ##",
"# # # ### ### ####### ### # ####### # ######### ##",
"# # #   # #           # # # #     # #   #       ##",
"# # ### # # ########### # ### ### # ### # ########",
"# #     # # #   #     #   #   #   # #   # #     ##",
"# ####### ### # # ### # ### ### ##### ### # ######",
"#   #     #   # #   #   #     #       # # #     ##",
"### # ##### ### ####### # ### ######### # # ### ##",
"# # #       #   #     # # #   #     #     #   # ##",
"# # # ####### ### ### # ### ### ### # ####### # ##",
"#   # #     #   #   # #   # #   #   # #     # # ##",
"# ### # ### ### # ### ### # # # ##### # ### # # ##",
"# #   #   #   #   #   # #   # #   #   #   # # # ##",
"# # ##### ### ### # ### ######### # ####### # # ##",
"# # #   #   #   # # #   #         # #     #   # ##",
"# # ### ### ### ### # ### # ##### # ### # ##### ##",
"# #     #   #   #   # #   # #     # #   #       ##",
"# ####### ### ### ### # ### ####### # ######### ##",
"#         #       #       #           #        G##",
"##################################################"
    };

    int n = grid.size();
    int m = grid[0].size();

    // Create a refractive index grid for each cell.
    std::vector<std::vector<double>> refractive(n, std::vector<double>(m, 1.0));
    // Optionally initialize a denser region (will be updated dynamically later).
    for (int i = 3; i < 6 && i < n; i++) {
        for (int j = 10; j < 18 && j < m; j++) {
            if (grid[i][j] == ' ')
                refractive[i][j] = 1.5;
        }
    }

    // Locate the start (S) and goal (G) positions in the grid.
    std::pair<int, int> start, goal;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            if (grid[i][j] == 'S')
                start = { i, j };
            else if (grid[i][j] == 'G')
                goal = { i, j };
        }
    }

    // Set up the cost array and parent pointers for path reconstruction.
    std::vector<std::vector<double>> dist(n, std::vector<double>(m, std::numeric_limits<double>::infinity()));
    std::vector<std::vector<std::pair<int, int>>> parent(n, std::vector<std::pair<int, int>>(m, { -1, -1 }));

    // Advanced heuristic: the Euclidean distance weighted by an average refractive index.
    auto heuristic = [&](int i, int j) {
        double dx = i - goal.first;
        double dy = j - goal.second;
        double distance = std::sqrt(dx * dx + dy * dy);
        double avgIndex = sampleAverageRefractive(i, j, goal.first, goal.second, refractive);
        return distance * avgIndex;
        };

    // Start the timer before running the search.
    auto searchStart = std::chrono::high_resolution_clock::now();

    // Update the dynamic refractive indices using the current time.
    double currentTime = std::chrono::duration<double>(searchStart.time_since_epoch()).count();
    updateRefractiveIndices(grid, refractive, currentTime);

    // Priority queue for A* search.
    std::priority_queue<Node, std::vector<Node>, CompareNode> pq;
    dist[start.first][start.second] = 0;
    pq.push({ start.first, start.second, 0, heuristic(start.first, start.second) });

    // Possible movements: up, down, left, right.
    std::vector<std::pair<int, int>> directions = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };

    // A* search loop.
    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        // If the goal is reached, exit the loop.
        if (current.x == goal.first && current.y == goal.second)
            break;

        // If a better path to this node has already been found, skip it.
        if (current.cost > dist[current.x][current.y])
            continue;

        // Explore neighboring cells.
        for (auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;

            // Skip if out of bounds or if the cell is a wall.
            if (nx < 0 || ny < 0 || nx >= n || ny >= m || grid[nx][ny] == '#')
                continue;

            // Calculate new cost based on the refractive index.
            double newCost = current.cost + refractive[nx][ny];
            if (newCost < dist[nx][ny]) {
                dist[nx][ny] = newCost;
                parent[nx][ny] = { current.x, current.y };
                double priority = newCost + heuristic(nx, ny);
                pq.push({ nx, ny, newCost, priority });
            }
        }
    }

    // Stop the timer after the search completes.
    auto searchEnd = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedSearch = searchEnd - searchStart;  // in seconds

    // Check if a valid path was found.
    if (dist[goal.first][goal.second] == std::numeric_limits<double>::infinity()) {
        std::cout << "No path found" << std::endl;
        return 0;
    }

    // Reconstruct the discrete path from goal to start.
    std::vector<std::pair<int, int>> discretePath;
    std::pair<int, int> cur = goal;
    while (cur != start) {
        discretePath.push_back(cur);
        cur = parent[cur.first][cur.second];
    }
    discretePath.push_back(start);
    std::reverse(discretePath.begin(), discretePath.end());

    // Mark the discrete path on the grid for visualization.
    for (auto& pt : discretePath) {
        if (grid[pt.first][pt.second] == ' ')
            grid[pt.first][pt.second] = '.';
    }

    // Optionally, refine the path in continuous space to smooth it.
    std::vector<std::pair<double, double>> refinedPath = refinePath(discretePath, refractive);

    // Output the grid with the discrete path and timing information.
    for (const auto& row : grid) {
        std::cout << row << std::endl;
    }
    std::cout << "\nDiscrete path found in " << elapsedSearch.count() << " seconds" << std::endl;

    // Output the refined continuous path coordinates.
    std::cout << "\nRefined continuous path coordinates:" << std::endl;
    for (auto& pt : refinedPath) {
        std::cout << "(" << pt.first << ", " << pt.second << ") ";
    }
    std::cout << std::endl;

    return 0;
}
