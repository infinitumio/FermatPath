#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <algorithm>
#include <chrono>

using namespace std;
using namespace std::chrono;

// Structure to represent a point in the maze.
struct Point {
    int x, y;
};

// Helper function: Check if a coordinate is within the maze bounds.
bool inBounds(int x, int y, int cols, int rows) {
    return (x >= 0 && x < cols && y >= 0 && y < rows);
}

// Node structure for BFS that stores the cell position and a pointer to its parent.
struct Node {
    int x, y;
    Node* parent;
};

int main() {
    // Define the maze (walls are '#', free cells are ' ', start 'S', and goal 'G')
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

    // Identify the endpoints: start (S) and goal (G).
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

    // Start timing the BFS execution.
    auto startTime = high_resolution_clock::now();

    // Prepare a 2D visited array.
    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    // Queue for BFS.
    queue<Node*> q;

    // Create the start node and enqueue it.
    Node* startNode = new Node{ start.x, start.y, nullptr };
    q.push(startNode);
    visited[start.y][start.x] = true;

    Node* finalNode = nullptr;
    // Define 4-connected movement: right, left, down, up.
    int dirs[4][2] = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };

    // BFS loop.
    while (!q.empty()) {
        Node* current = q.front();
        q.pop();

        // Check if we've reached the goal.
        if (current->x == goal.x && current->y == goal.y) {
            finalNode = current;
            break;
        }

        // Explore neighbors.
        for (auto& d : dirs) {
            int nx = current->x + d[0];
            int ny = current->y + d[1];
            if (inBounds(nx, ny, cols, rows) && maze[ny][nx] != '#' && !visited[ny][nx]) {
                visited[ny][nx] = true;
                Node* neighbor = new Node{ nx, ny, current };
                q.push(neighbor);
            }
        }
    }

    // Reconstruct the path from goal to start using parent pointers.
    vector<Point> path;
    Node* node = finalNode;
    while (node != nullptr) {
        path.push_back({ node->x, node->y });
        node = node->parent;
    }
    reverse(path.begin(), path.end());

    // Stop timing here.
    auto endTime = high_resolution_clock::now();
    double elapsedTime = duration<double>(endTime - startTime).count();

    // Create a copy of the maze and overlay the path (marking with '+')
    vector<string> mazePath = maze;
    for (auto& p : path) {
        if (mazePath[p.y][p.x] != 'S' && mazePath[p.y][p.x] != 'G')
            mazePath[p.y][p.x] = '+';
    }

    // Output the maze with the path and the elapsed time.
    cout << "Final Path Reconstruction (BFS):" << endl;
    for (auto& row : mazePath) {
        cout << row << endl;
    }
    cout << "\nElapsed time: " << elapsedTime << " seconds" << endl;

    // Note: In production code, you should free all allocated nodes.
    // For brevity, cleanup is omitted as the program is ending.

    return 0;
}
