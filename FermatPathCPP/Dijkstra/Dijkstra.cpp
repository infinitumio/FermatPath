#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <limits>
#include <algorithm>
#include <chrono>

using namespace std;
using namespace std::chrono;

// Structure to represent a point in the maze.
struct Point {
    int x, y;
};

// Check if a coordinate is within the maze bounds.
bool inBounds(int x, int y, int cols, int rows) {
    return (x >= 0 && x < cols && y >= 0 && y < rows);
}

// Node structure for Dijkstra's algorithm.
struct Node {
    int x, y;       // Position in the maze.
    int cost;       // Cumulative cost from start.
    Node* parent;   // Pointer to the parent node.
};

// Comparator for the priority queue (min-heap based on cost).
struct NodeComparator {
    bool operator()(const Node* a, const Node* b) const {
        return a->cost > b->cost;
    }
};

int main() {
    // Define the maze.
    vector<string> maze = {
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

    int rows = maze.size();
    int cols = maze[0].size();

    // Locate the start (S) and goal (G) points.
    Point start, goal;
    bool foundStart = false, foundGoal = false;
    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            if (maze[y][x] == 'S') {
                start = { x, y };
                foundStart = true;
            }
            else if (maze[y][x] == 'G') {
                goal = { x, y };
                foundGoal = true;
            }
        }
    }
    if (!foundStart || !foundGoal) {
        cout << "Maze must contain both S (start) and G (goal)!" << endl;
        return 1;
    }

    // Start timing.
    auto startTime = high_resolution_clock::now();

    // Prepare data structures for Dijkstra's algorithm.
    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    // Cost from start for each cell.
    vector<vector<int>> cost(rows, vector<int>(cols, numeric_limits<int>::max()));
    priority_queue<Node*, vector<Node*>, NodeComparator> open;

    // Create the start node.
    Node* startNode = new Node{ start.x, start.y, 0, nullptr };
    cost[start.y][start.x] = 0;
    open.push(startNode);

    Node* finalNode = nullptr;
    // 4-connected neighbors: right, left, down, up.
    int dirs[4][2] = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };

    // Dijkstra's algorithm loop.
    while (!open.empty()) {
        Node* current = open.top();
        open.pop();

        // If we've reached the goal, finish.
        if (current->x == goal.x && current->y == goal.y) {
            finalNode = current;
            break;
        }

        // Skip if this cell has been processed.
        if (visited[current->y][current->x])
            continue;
        visited[current->y][current->x] = true;

        // Explore neighbors.
        for (auto& d : dirs) {
            int nx = current->x + d[0];
            int ny = current->y + d[1];
            if (inBounds(nx, ny, cols, rows) && maze[ny][nx] != '#') {
                int newCost = current->cost + 1; // Each move costs 1.
                if (newCost < cost[ny][nx]) {
                    cost[ny][nx] = newCost;
                    Node* neighbor = new Node{ nx, ny, newCost, current };
                    open.push(neighbor);
                }
            }
        }
    }

    // Reconstruct the path from goal to start.
    vector<Point> path;
    Node* node = finalNode;
    while (node != nullptr) {
        path.push_back({ node->x, node->y });
        node = node->parent;
    }
    reverse(path.begin(), path.end());

    // Stop timing.
    auto endTime = high_resolution_clock::now();
    double elapsedTime = duration<double>(endTime - startTime).count();

    // Visualize the maze with the overlaid path.
    vector<string> mazePath = maze;
    for (auto& p : path) {
        if (mazePath[p.y][p.x] != 'S' && mazePath[p.y][p.x] != 'G')
            mazePath[p.y][p.x] = '+';
    }

    cout << "Final Path Reconstruction (Dijkstra):" << endl;
    for (auto& row : mazePath) {
        cout << row << endl;
    }
    cout << "\nElapsed time: " << elapsedTime << " seconds" << endl;

    // Cleanup: In a production system, you'd free all allocated nodes.
    // Since the program is ending, we omit full cleanup here.

    return 0;
}
