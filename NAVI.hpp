#include "vex.h"

using namespace vex;

const int GRID_SIZE = 36;
const int OFFSET = 18;

int startX, startY;
int goalX, goalY;

bool walkable[GRID_SIZE][GRID_SIZE];
bool isPath[GRID_SIZE][GRID_SIZE] = { false };
double gridToMM(int gridconvers) {
  return gridconvers * 100.00;
}

int toGridCoord(double mm) {
  return static_cast<int>(round(mm / 100.00));
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
  for (int y = -OFFSET; y <= OFFSET; y += step) {
    for (int x = -OFFSET; x <= OFFSET; x += step) {
      int rx = x + OFFSET;
      int ry = y + OFFSET;
      if (x == 0 && y == 0) Brain.Screen.print("X");
      else if (x == startX && y == startY) Brain.Screen.print("R");
      else if (x == goalX && y == goalY) Brain.Screen.print("Z");
      else if (!walkable[ry][rx]) Brain.Screen.print("#");
      else if (isPath[ry][rx]) Brain.Screen.print("*");
      else Brain.Screen.print(".");
    }
    Brain.Screen.newLine();
  }
}




void turnTo(double targetAngleDeg) {
  double current = Gps2.heading();
  double error = targetAngleDeg - current;
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  double direction = (error > 0) ? 1 : -1;
  while (fabs(error) > 3) {
    LeftDrivetrain.spin(forward, 20 * direction, percent);
    RightDrivetrain.spin(reverse, 20 * direction, percent);
    wait(50, msec);
    current = Gps2.heading();
    error = targetAngleDeg - current;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    direction = (error > 0) ? 1 : -1;
  }
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
//heuristic ist richtig fehler ist im grid system 
int heuristic(int x1, int y1, int x2, int y2) {
  return 10 * (abs(x1 - x2) + abs(y1 - y2));
}

void calculatePath() {
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      nodes[y][x] = new Node{x - OFFSET, y - OFFSET};
      nodes[y][x]->gCost = 9999;
      nodes[y][x]->hCost = 9999;
      nodes[y][x]->parent = nullptr;
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

  int dx[8] = { -1, -1,  0, 1, 1,  1,  0, -1 };
  int dy[8] = {  0, -1, -1, -1, 0,  1,  1,  1 };

  while (!openSet.empty()) {
    Node* current = openSet[0];
    int currentIndex = 0;
    for (int i = 1; i < openSet.size(); i++) {
      if (openSet[i]->fCost() < current->fCost()) {
        current = openSet[i];
        currentIndex = i;
      }
    }
    openSet.erase(openSet.begin() + currentIndex);
    int cx = current->x + OFFSET;
    int cy = current->y + OFFSET;
    closedSet[cy][cx] = true;

    if (current == goal) {
      Node* p = goal;
      while (p != start && p != nullptr) {
        int gx = p->x + OFFSET;
        int gy = p->y + OFFSET;
        isPath[gy][gx] = true;
        pathWaypoints.push_back({p->x, p->y});
        p = p->parent;
      }
      std::reverse(pathWaypoints.begin(), pathWaypoints.end());
      return;
    }

    for (int d = 0; d < 8; d++) {
      int nx = current->x + dx[d];
      int ny = current->y + dy[d];
      if (nx < -OFFSET || nx > OFFSET || ny < -OFFSET || ny > OFFSET) continue;
      int rx = nx + OFFSET;
      int ry = ny + OFFSET;
      if (!walkable[ry][rx] || closedSet[ry][rx]) continue;

      if (dx[d] != 0 && dy[d] != 0) {
        int adj1x = current->x + dx[d];
        int adj1y = current->y;
        int adj2x = current->x;
        int adj2y = current->y + dy[d];
        if (!walkable[adj1y + OFFSET][adj1x + OFFSET] ||
            !walkable[adj2y + OFFSET][adj2x + OFFSET]) continue;
      }

      Node* neighbor = nodes[ry][rx];
      int moveCost = (dx[d] == 0 || dy[d] == 0) ? 10 : 14;
      int tentativeG = current->gCost + moveCost;

      if (tentativeG < neighbor->gCost) {
        neighbor->parent = current;
        neighbor->gCost = tentativeG;
        neighbor->hCost = heuristic(nx, ny, goalX, goalY);
        openSet.push_back(neighbor);
      }
    }
  }
}

void followPath() {
  for (auto& point : pathWaypoints) {
    int targetX = point.first;
    int targetY = point.second;
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
  if (targetX < -OFFSET) targetX = -OFFSET;
  if (targetX > OFFSET)  targetX = OFFSET;
  if (targetY < -OFFSET) targetY = -OFFSET;
  if (targetY > OFFSET)  targetY = OFFSET;
  goalX = targetX;
  goalY = targetY;

  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      walkable[y][x] = true;
      isPath[y][x] = false;
    }
  }

  int obstacles[][2] = {
    {-6, -6}, {6, 6}, {-6, 6}, {6, -6}
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

