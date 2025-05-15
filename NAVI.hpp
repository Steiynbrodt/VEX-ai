#include "vex.h"
using namespace vex;


// Field dimensions
const double FIELD_SIZE_MM = 3600.0;  // Field is 3600mm x 3600mm

// Use a 37x37 grid to have a unique center cell (0,0)
const int GRID_SIZE = 37;
const int OFFSET = 18;  // Grid coordinate = index - OFFSET, so valid range: -18 to +18

// Calculate cell size so that the grid exactly covers the field.
const double CELL_SIZE = FIELD_SIZE_MM / GRID_SIZE;  // ≈ 97.30 mm per cell

int startX, startY;
int goalX, goalY;

bool walkable[GRID_SIZE][GRID_SIZE];
bool isPath[GRID_SIZE][GRID_SIZE] = { false };

void Debuging(){
  while(true){
  Brain.Screen.print(INS.heading());
  Brain.Screen.newLine();
  }
  
}
// Converts a grid coordinate to millimeters.
double gridToMM(int gridVal) {
  return gridVal * CELL_SIZE;
}

// Converts a millimeter measurement to a grid coordinate.
int toGridCoord(double mm) {
  return static_cast<int>(round(mm / CELL_SIZE));
}

void updateStartPositionFromGPS() {
  startX = toGridCoord(Gps2.xPosition(mm));
  startY = toGridCoord(Gps2.yPosition(mm));
}

std::vector<std::pair<int, int>> pathWaypoints;

void printGrid() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  const int step = 3;
  // Loop over grid coordinates from -18 to +18
  for (int y = -OFFSET; y <= OFFSET; y += step) {
    for (int x = -OFFSET; x <= OFFSET; x += step) {
      int rx = x + OFFSET;
      int ry = y + OFFSET;
      if (x == 0 && y == 0)
        Brain.Screen.print("X");
      else if (x == startX && y == startY)
        Brain.Screen.print("R");
      else if (x == goalX && y == goalY)
        Brain.Screen.print("Z");
      else if (!walkable[ry][rx])
        Brain.Screen.print("#");
      else if (isPath[ry][rx])
        Brain.Screen.print("*");
      else
        Brain.Screen.print(".");
    }
    Brain.Screen.newLine();
  }
}

void turnTo(int targetAngleDeg) {
  // Initial sensor reading and error calculation.
  double currentangle = Gps1.heading()+180.00;// it doesnt konw what its heading in the fields cooridnate field is needs  to be fixed is fixed should now add 180 to  gps value to get positive outputs  instead of -180 to 180 values
  double error = targetAngleDeg - currentangle;
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  double direction = (error > 0) ? 1 : -1;
  
  // Continue turning until within 10 degrees of the target angle.
  while (fabs(error) > 15) {
    // Spin the drivetrain with a constant speed.
    LeftDrivetrain.spin(forward, -5 * direction, percent);
    RightDrivetrain.spin(reverse, -5 * direction, percent);
    
    // Wait a moment to allow sensors to update.
    wait(50, msec);

    // Update the current heading and recalculate the error.
    double currentangle = Gps1.heading()+180.00;
    error = targetAngleDeg - currentangle;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    
    // Determine direction based on the updated error.
    direction = (error > 0) ? 1 : -1;
  }
  
  // Stop the drivetrain after the turn is complete.
  LeftDrivetrain.stop(brake);
  RightDrivetrain.stop(brake);
}

void driveTo(double dxmm, double dymm) {
  double dist = sqrt(dxmm * dxmm + dymm * dymm);
  double angle = atan2(dymm, dxmm) * 180.0 / M_PI;
  double wheelCirc = 26.732 * M_PI;
  double degreesToSpin = (dist / wheelCirc) * 360.0;
  turnTo(fmod(angle + 360, 360));
  driveMotorLeftOne.spinFor(forward, degreesToSpin, degrees, false);
  driveMotorLeftTwo.spinFor(forward, degreesToSpin, degrees, false);
  driveMotorLeftThree.spinFor(forward, degreesToSpin, degrees, false);
  driveMotorRightOne.spinFor(forward, degreesToSpin, degrees, false);
  driveMotorRightTwo.spinFor(forward, degreesToSpin, degrees, false);
  driveMotorRightThree.spinFor(forward, degreesToSpin, degrees, true);
}

struct Node {
  int x, y;
  int gCost, hCost;
  Node* parent;
  int fCost() const { return gCost + hCost; }
};

Node* nodes[GRID_SIZE][GRID_SIZE];

// Manhattan distance heuristic
int heuristic(int x1, int y1, int x2, int y2) {
  return 10 * (abs(x1 - x2) + abs(y1 - y2));
}

void calculatePath() {
  // Allocate and initialize nodes for each grid cell.
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      nodes[y][x] = new Node{x - OFFSET, y - OFFSET, 9999, 9999, nullptr};
      isPath[y][x] = false;
    }
  }
  
  pathWaypoints.clear();
  
  Node* start = nodes[startY + OFFSET][startX + OFFSET];
  Node* goal = nodes[goalY + OFFSET][goalX + OFFSET];
  start->gCost = 0;
  start->hCost = heuristic(startX, startY, goalX, goalY);
  
  std::vector<Node*> openSet = { start };
  bool closedSet[GRID_SIZE][GRID_SIZE] = { false };
  
  int dx[8] = { -1, -1, 0, 1, 1, 1, 0, -1 };
  int dy[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };
  
  while (!openSet.empty()) {
    Node* currentnode = openSet[0];
    int currentIndex = 0;
    for (int i = 1; i < openSet.size(); i++) {
      if (openSet[i]->fCost() < currentnode->fCost()) {
        currentnode = openSet[i];
        currentIndex = i;
      }
    }
    openSet.erase(openSet.begin() + currentIndex);
    int cx = currentnode->x + OFFSET;
    int cy = currentnode->y + OFFSET;
    closedSet[cy][cx] = true;
    
    // If the goal is reached, backtrack to record the path.
    if (currentnode == goal) {
      Node* p = goal;
      while (p != start && p != nullptr) {
        int gx = p->x + OFFSET;
        int gy = p->y + OFFSET;
        isPath[gy][gx] = true;
        pathWaypoints.push_back({p->x, p->y});
        p = p->parent;
      }
      std::reverse(pathWaypoints.begin(), pathWaypoints.end());
      break;
    }
    
    // Process neighbors.
    for (int d = 0; d < 8; d++) {
      int nx = currentnode->x + dx[d];
      int ny = currentnode->y + dy[d];
      if (nx < -OFFSET || nx > OFFSET || ny < -OFFSET || ny > OFFSET) continue;
      int rx = nx + OFFSET;
      int ry = ny + OFFSET;
      if (!walkable[ry][rx] || closedSet[ry][rx]) continue;
      
      // For diagonal movement, ensure adjacent cells are walkable.
      if (dx[d] != 0 && dy[d] != 0) {
        int adj1x = currentnode->x + dx[d];
        int adj1y = currentnode->y;
        int adj2x = currentnode->x;
        int adj2y = currentnode->y + dy[d];
        if (!walkable[adj1y + OFFSET][adj1x + OFFSET] ||
            !walkable[adj2y + OFFSET][adj2x + OFFSET]) continue;
      }
      
      Node* neighbor = nodes[ry][rx];
      int moveCost = (dx[d] == 0 || dy[d] == 0) ? 10 : 14;
      int tentativeG = currentnode->gCost + moveCost;
      
      if (tentativeG < neighbor->gCost) {
        neighbor->parent = currentnode;
        neighbor->gCost = tentativeG;
        neighbor->hCost = heuristic(nx, ny, goalX, goalY);
        openSet.push_back(neighbor);
      }
    }
  }
  
  // Free the dynamically allocated nodes to avoid memory leaks.
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      delete nodes[y][x];
      nodes[y][x] = nullptr;
    }
  }
}

void followPath() {
  for (auto& point : pathWaypoints) {
    int targetX = point.first;
    int targetY = point.second;
    // Compute movement in millimeters based on grid coordinate differences.
    double dx = gridToMM(targetX - startX);
    double dy = gridToMM(targetY - startY);
    driveTo(dx, dy);
    wait(300, msec);
    updateStartPositionFromGPS();
    startX = toGridCoord(Gps2.xPosition(mm));
    startY = toGridCoord(Gps2.yPosition(mm));
  }
}

int NAVI(int targetX, int targetY) {
  // Clamp target coordinates to valid grid coordinates.
  if (targetX < -OFFSET) targetX = -OFFSET;
  if (targetX > OFFSET)  targetX = OFFSET;
  if (targetY < -OFFSET) targetY = -OFFSET;
  if (targetY > OFFSET)  targetY = OFFSET;
  goalX = targetX;
  goalY = targetY;
  
  // Initialize grid: mark all cells as walkable and clear any previous path markings.
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      walkable[y][x] = true;
      isPath[y][x] = false;
    }
  }
  
  // Example obstacles (grid coordinates).
  int obstacles[][2] = {
    {-6, 0}, {6, 0}, {0, 6}, {0, -6},{-7, 0}, {7, 0}, {0, -7}, {7, 0},{-6, -1}, {6, 1}, {1, 6}, {1, -6},{-5, 0}, {5, 0}, {0, 5}, {0, -5}
    
  };
  int obstacleCount = sizeof(obstacles) / sizeof(obstacles[0]);
  for (int i = 0; i < obstacleCount; i++) {
    int ox = obstacles[i][0];
    int oy = obstacles[i][1];
    if (ox >= -OFFSET && ox <= OFFSET && oy >= -OFFSET && oy <= OFFSET) {
      walkable[oy + OFFSET][ox + OFFSET] = false;
    }
  }
  
  updateStartPositionFromGPS();
  calculatePath();
  printGrid();
  followPath();
  

  while (true) {
    wait(500, msec);
  }
}


