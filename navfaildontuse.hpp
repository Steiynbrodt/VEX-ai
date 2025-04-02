#include "vex.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <queue>
#include <fstream>
#include <limits>
//------------------- POTENTIAL FIELDS NAVIGATION -------------------//

// Drives toward a target coordinate using potential fields.
// Uses fixed tuning parameters K_att and K_rep.
void potentialFieldsNavigate(double targetX, double targetY, double maxSpeed = 10.00) {
    const double K_att = 1.9;   // Attractive force gain
    const double K_rep = 10000; // Repulsive force gain

    while (true) {
        double X, Y, Heading;
        getGPSPosition(X, Y, Heading);
        
        double deltaX = targetX - X;
        double deltaY = targetY - Y;
        double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
        
        if (distance < TOLERANCE) {
            FullDrivetrain.stop();
            break;
        }
        
        // Attractive force toward target.
        double F_att_x = K_att * deltaX;
        double F_att_y = K_att * deltaY;
        
        // Initialize repulsive forces.
        double F_rep_x = 0.0;
        double F_rep_y = 0.0;
        
        // Repulsive force from walls (assume walls at ±1800 mm)
        if (X < (-1800 + WALL_THRESHOLD)) {
            double d = fabs(X - (-1800));
            F_rep_x += K_rep / (d * d);
        }
        if (X > (1800 - WALL_THRESHOLD)) {
            double d = fabs(X - 1800); // Adjust if necessary.
            F_rep_x -= K_rep / (d * d);
        }
        if (Y < (-1800 + WALL_THRESHOLD)) {
            double d = fabs(Y - (-1800));
            F_rep_y += K_rep / (d * d);
        }
        if (Y > (1800 - WALL_THRESHOLD)) {
            double d = fabs(Y - 1800);
            F_rep_y -= K_rep / (d * d);
        }
        
        // Repulsive force from posts.
        for (int i = 0; i < 4; i++) {
            double dx = X - postsArr[i].x;
            double dy = Y - postsArr[i].y;
            double d = sqrt(dx * dx + dy * dy);
            if (d < OBSTACLE_THRESHOLD && d > 0) {
                double repForce = K_rep / (d * d);
                F_rep_x += repForce * (dx / d);
                F_rep_y += repForce * (dy / d);
            }
        }
        
        // Total force vector.
        double F_total_x = F_att_x + F_rep_x;
        double F_total_y = F_att_y + F_rep_y;
        
        // Desired heading based on total force.
        double desiredHeading = atan2(F_total_y, F_total_x) * 180.0 / M_PI;
        double headingError = desiredHeading - Heading;
        if (headingError > 180) headingError -= 360;
        if (headingError < -180) headingError += 360;
        
        // Proportional control for turning and driving.
        double turnPower = std::max(std::min(headingError * 0.1, 20.0), -20.0);
        double driveSpeed = std::max(std::min(distance * 0.1, 10.00), 30.00);
        
        LeftDrivetrain.spin(forward, driveSpeed - turnPower, percent);
        RightDrivetrain.spin(forward, driveSpeed + turnPower, percent);
        
        if (isStuck()) {
            avoidObstacle();
        }
        
        task::sleep(50);
    }
}

//------------------- GRID-BASED NAVIGATION (WITHOUT POLYGONS) -------------------//

// We'll build an occupancy grid based solely on posts.
// grid[i][j] is marked true if the cell's center is too close to any post.
bool grid[GRID_SIZE][GRID_SIZE];  // true = occupied

void initGrid() {
  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      grid[i][j] = false;
      double cellX = MIN_FIELD + (i + 0.5) * CELL_SIZE;
      double cellY = MIN_FIELD + (j + 0.5) * CELL_SIZE;
      // Mark cell as occupied if too close to any post.
      for (int k = 0; k < 4; k++) {
        double dx = cellX - postsArr[k].x;
        double dy = cellY - postsArr[k].y;
        if (sqrt(dx * dx + dy * dy) < OBSTACLE_THRESHOLD) {
          grid[i][j] = true;
          break;
        }
      }
    }
  }
}

//------------------- A* PATH PLANNING -------------------//
struct Node {
  int x, y;
  double g, h, f;
  int parentX, parentY;
  bool closed;
};

Node nodes[GRID_SIZE][GRID_SIZE];

double heuristic(int x, int y, int goalX, int goalY) {
  return sqrt(pow(x - goalX, 2) + pow(y - goalY, 2));
}

std::vector<std::pair<int,int>> aStar(int startX, int startY, int goalX, int goalY) {
  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      nodes[i][j].x = i;
      nodes[i][j].y = j;
      nodes[i][j].g = std::numeric_limits<double>::infinity();
      nodes[i][j].h = heuristic(i, j, goalX, goalY);
      nodes[i][j].f = std::numeric_limits<double>::infinity();
      nodes[i][j].parentX = -1;
      nodes[i][j].parentY = -1;
      nodes[i][j].closed = false;
    }
  }
  nodes[startX][startY].g = 0;
  nodes[startX][startY].f = nodes[startX][startY].h;
  
  std::vector<std::pair<int,int>> openList;
  openList.push_back({startX, startY});
  
  while (!openList.empty()) {
    int currentIndex = 0;
    for (int i = 1; i < openList.size(); i++) {
      int cx = openList[i].first, cy = openList[i].second;
      int bestx = openList[currentIndex].first, besty = openList[currentIndex].second;
      if (nodes[cx][cy].f < nodes[bestx][besty].f)
        currentIndex = i;
    }
    int currentX = openList[currentIndex].first;
    int currentY = openList[currentIndex].second;
    openList.erase(openList.begin() + currentIndex);
    nodes[currentX][currentY].closed = true;
    
    if (currentX == goalX && currentY == goalY) {
      std::vector<std::pair<int,int>> path;
      int x = goalX, y = goalY;
      while (!(x == startX && y == startY)) {
        path.push_back({x, y});
        int px = nodes[x][y].parentX;
        int py = nodes[x][y].parentY;
        x = px; y = py;
      }
      path.push_back({startX, startY});
      std::reverse(path.begin(), path.end());
      return path;
    }
    
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        int nx = currentX + dx, ny = currentY + dy;
        if (nx < 0 || nx >= GRID_SIZE || ny < 0 || ny >= GRID_SIZE) continue;
        if (grid[nx][ny]) continue;
        if (nodes[nx][ny].closed) continue;
        
        double cost = (dx != 0 && dy != 0) ? 1.414 : 1.0;
        double tentativeG = nodes[currentX][currentY].g + cost;
        if (tentativeG < nodes[nx][ny].g) {
          nodes[nx][ny].parentX = currentX;
          nodes[nx][ny].parentY = currentY;
          nodes[nx][ny].g = tentativeG;
          nodes[nx][ny].f = tentativeG + nodes[nx][ny].h;
          bool inOpen = false;
          for (auto &p : openList) {
            if (p.first == nx && p.second == ny) { inOpen = true; break; }
          }
          if (!inOpen)
            openList.push_back({nx, ny});
        }
      }
    }
  }
  return std::vector<std::pair<int,int>>();  // Return empty path if none found.
}

// Convert grid cell indices to field coordinates.
void gridCellToFieldCoord(int i, int j, double &x, double &y) {
  x = MIN_FIELD + (i + 0.5) * CELL_SIZE;
  y = MIN_FIELD + (j + 0.5) * CELL_SIZE;
}

// Convert a field coordinate to a grid cell index.
int fieldCoordToGridIndex(double coord) {
  int index = (int)((coord - MIN_FIELD) / CELL_SIZE);
  if (index < 0) index = 0;
  if (index >= GRID_SIZE) index = GRID_SIZE - 1;
  return index;
}

// Grid-based navigation: plan a path using A* and drive through the waypoints.
void gridBasedNavigate(double targetX, double targetY) {
  initGrid();
  
  double curX, curY, curHeading;
  getGPSPosition(curX, curY, curHeading);
  int startI = fieldCoordToGridIndex(curX);
  int startJ = fieldCoordToGridIndex(curY);
  int goalI = fieldCoordToGridIndex(targetX);
  int goalJ = fieldCoordToGridIndex(targetY);
  
  std::vector<std::pair<int,int>> path = aStar(startI, startJ, goalI, goalJ);
  if (path.empty()) {
    Brain.Screen.print("Path not found!");
    return;
  }
  
  for (auto &cell : path) {
    double wx, wy;
    gridCellToFieldCoord(cell.first, cell.second, wx, wy);
    potentialFieldsNavigate(wx, wy);
    task::sleep(200);
  }
}
void vertex_grid(){
  
}
//------------------- HARDWARE FUNCTIONS -------------------//

#include <iostream>
#include <vector>

const int GRID_SIZE = 37;
const int OFFSET = 18;

class GITTER {
private:
    bool walkable[GRID_SIZE][GRID_SIZE];  // true = begehbar, false = Hindernis

public:
    GITTER() {
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                walkable[y][x] = true;
            }
        }
    }

    void setWalkable(int x, int y, bool isWalkable) {
        if (inBounds(x, y)) {
            walkable[y + OFFSET][x + OFFSET] = isWalkable;
        }
    }

    bool isWalkable(int x, int y) const {
        if (inBounds(x, y)) {
            return walkable[y + OFFSET][x + OFFSET];
        }
        return false;
    }

    bool inBounds(int x, int y) const {
        return (x >= -OFFSET && x <= OFFSET && y >= -OFFSET && y <= OFFSET);
    }

    std::vector<std::pair<int, int>> getNeighbors(int x, int y) const {
        std::vector<std::pair<int, int>> neighbors;

        int dx[8] = { -1,  0, 1, 0, -1, -1, 1, 1 };
        int dy[8] = {  0, -1, 0, 1, -1,  1, -1, 1 };

        for (int i = 0; i < 8; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            if (inBounds(nx, ny) && isWalkable(nx, ny)) {
                neighbors.emplace_back(nx, ny);
            }
        }

        return neighbors;
    }

    void printGrid() const {
        for (int y = -OFFSET; y <= OFFSET; y++) {
            for (int x = -OFFSET; x <= OFFSET; x++) {
                std::cout << (isWalkable(x, y) ? "." : "#") << " ";
            }
            std::cout << "\n";
        }
    }
};


int NAVFAIL() {
    GITTER gitter;

    // Beispiel: Setze ein Hindernis bei (2, 3)
    gitter.setWalkable(2, 3, false);

    // Überprüfe, ob (2,3) begehbar ist
    if (!gitter.isWalkable(2, 3)) {
        std::cout << "(2,3) ist ein Hindernis\n";
    }

    // Hole Nachbarn von (0,0)
    auto neighbors = gitter.getNeighbors(0, 0);
    std::cout << "Begehbare Nachbarn von (0,0):\n";
    for (const auto& n : neighbors) {
        std::cout << "(" << n.first << "," << n.second << ")\n";
    }

    // Gitter anzeigen
    std::cout << "\nGitteransicht:\n";
    gitter.printGrid();

    return 0;
}