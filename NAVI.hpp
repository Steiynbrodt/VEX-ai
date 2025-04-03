#include "vex.h"

using namespace vex;

const int GRID_SIZE = 37;
const int OFFSET = 18;

// Roboterposition und Ziel
int startX,startY;
int goalX,goalY;

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

// Gitter: begehbar und Pfadmarkierung
bool walkable[GRID_SIZE][GRID_SIZE];
bool isPath[GRID_SIZE][GRID_SIZE] = { false };

void printGrid() {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
  
    const int step = 4;  // Nur jedes 2. Feld anzeigen für bessere Darstellung
  
    for (int y = -OFFSET; y <= OFFSET; y += step) {
      for (int x = -OFFSET; x <= OFFSET; x += step) {
        int rx = x + OFFSET;
        int ry = y + OFFSET;
  
        if (x == 0 && y == 0) {
          Brain.Screen.print("X");
        } else if (x == startX && y == startY) {
          Brain.Screen.print("R");
        } else if (x == goalX && y == goalY) {
          Brain.Screen.print("Z");
        } else if (!walkable[ry][rx]) {
          Brain.Screen.print("#");
        } else if (isPath[ry][rx]) {
          Brain.Screen.print("*");
        } else {
          Brain.Screen.print(".");
        }
      }
      Brain.Screen.newLine();
    }
  }

// Konvertierung auf tatsächliche Feldgröße: 3,7 m → 3700 mm über 37 Felder = 100 mm pro Feld
double gridToMM(int gridconvers) {
  return gridconvers * 100.0;
}

int toGridCoord(double mm) {
  return static_cast<int>(round(mm / 100.0));
}

void updateStartPositionFromGPS() {
  double gpsXmm = Gps2.xPosition(mm);
  double gpsYmm = Gps2.yPosition(mm);
  startX = toGridCoord(gpsXmm);
  startY = toGridCoord(gpsYmm);
}

void turnToAbsoluteAngle(double targetAngleDeg) {
  double current = Gps2.heading();
  double error = targetAngleDeg - current;

  // Normalize to [-180, 180]
  while (error > 180) error -= 360;
  while (error < -180) error += 360;

  double direction = (error > 0) ? 1 : -1;

  while (fabs(error) > 2) {
    LeftDrivetrain.spin(forward, 20 * direction, percent);
    RightDrivetrain.spin(reverse, 20 * direction, percent);
    wait(50, msec);
    current = Gps1.heading();
    error = targetAngleDeg - current;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    direction = (error > 0) ? 1 : -1;
  }

  LeftDrivetrain.stop(brake);
  RightDrivetrain.stop(brake);
}

struct Node {
  int x, y;
  int gCost, hCost;
  Node* parent;
  int fCost() const { return gCost + hCost; }
};

Node* findLowestFCost(Node* openSet[], int openCount) {
  Node* lowest = openSet[0];
  for (int i = 1; i < openCount; i++) {
    if (openSet[i]->fCost() < lowest->fCost()) {
      lowest = openSet[i];
    }
  }
  return lowest;
}

int heuristic(int x1, int y1, int x2, int y2) {
  return 10 * (abs(x1 - x2) + abs(y1 - y2));  // Manhattan distance scaled by 10
}

void calculatePath() {
  // Reset
  for (int y = 0; y < GRID_SIZE; y++) {
      for (int x = 0; x < GRID_SIZE; x++) {
          nodes[y][x] = new Node{x - OFFSET, y - OFFSET};
          nodes[y][x]->gCost = 9999;
          nodes[y][x]->hCost = 9999;
          nodes[y][x]->parent = nullptr;
          isPath[y][x] = false;
      }
  }

  Node* start = nodes[startY + OFFSET][startX + OFFSET];
  Node* goal = nodes[goalY + OFFSET][goalX + OFFSET];
  start->gCost = 0;
  start->hCost = heuristic(startX, startY, goalX, goalY);

  std::vector<Node*> openSet = { start };
  bool closedSet[GRID_SIZE][GRID_SIZE] = { false };

  // Directions: 8-way movement (N, NE, E, SE, S, SW, W, NW)
  int dx[8] = { -1, -1,  0, 1, 1,  1,  0, -1 };
  int dy[8] = {  0, -1, -1, -1, 0,  1,  1,  1 };

  while (!openSet.empty()) {
      // Find node with lowest fCost
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
              isPath[p->y + OFFSET][p->x + OFFSET] = true;
              p = p->parent;
          }
          return;
      }

      for (int d = 0; d < 8; d++) {
          int nx = current->x + dx[d];
          int ny = current->y + dy[d];

          if (nx < -OFFSET || nx > OFFSET || ny < -OFFSET || ny > OFFSET) continue;
          int rx = nx + OFFSET;
          int ry = ny + OFFSET;
          if (!walkable[ry][rx] || closedSet[ry][rx]) continue;

          // Prevent corner-cutting through walls
          if (dx[d] != 0 && dy[d] != 0) {
              int adj1x = current->x + dx[d];
              int adj1y = current->y;
              int adj2x = current->x;
              int adj2y = current->y + dy[d];

              if (!walkable[adj1y + OFFSET][adj1x + OFFSET] ||
                  !walkable[adj2y + OFFSET][adj2x + OFFSET]) {
                  continue;
              }
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
  int curX = startX;
  int curY = startY;

  while (!(curX == goalX && curY == goalY)) {
    updateStartPositionFromGPS();
    int newStartX = startX;
    int newStartY = startY;

    if (newStartX != curX || newStartY != curY) {
      curX = newStartX;
      curY = newStartY;
      calculatePath();
      //printGrid();
    }

    }

    bool moved = false;
    int dx[4] = { -1, 1, 0, 0 };
    int dy[4] = { 0, 0, -1, 1 };

    for (int d = 0; d < 4; d++) {
      int nx = curX + dx[d];
      int ny = curY + dy[d];

      if (nx < -OFFSET || nx > OFFSET || ny < -OFFSET || ny > OFFSET) continue;
      if (isPath[ny + OFFSET][nx + OFFSET]) {
        double dxmm = gridToMM(nx - curX);
        double dymm = gridToMM(ny - curY);
        double dist = sqrt(dxmm * dxmm + dymm * dymm);
        double angle = atan2(dymm, dxmm) * 180.0 / M_PI;
        double grees = (dist / (26.732 * M_PI)) * 360.0; 
        turnToAbsoluteAngle(fmod(angle + 360, 360));
        wait(100, msec);
        driveMotorLeftOne.spinFor(forward,grees, degrees, false);
        driveMotorLeftTwo.spinFor(forward,grees, degrees, false);
        driveMotorLeftThree.spinFor(forward,grees, degrees, false);
        driveMotorRightOne.spinFor(forward,grees, degrees, false);
        driveMotorRightTwo.spinFor(forward,grees, degrees, false);
        driveMotorRightThree.spinFor(forward,grees, degrees, true);
        // grobe Zeitschätzung basierend auf Distanz
        wait(100, msec);
        LeftDrivetrain.stop(brake);
        RightDrivetrain.stop(brake);
        curX = nx;
        curY = ny;
        moved = true;
        break;
      }
    }

    if (!moved) wait(100, msec);
  }


   

int NAVI(int targetX, int targetY) {
  // Ziel-Fallback: Begrenze Zielkoordinaten auf das gültige Gitter (-18 bis 18)
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

  // Hindernisse an festen Punkten setzen
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

