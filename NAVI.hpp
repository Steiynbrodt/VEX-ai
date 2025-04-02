#include "vex.h"

using namespace vex;

const int GRID_SIZE = 37;
const int OFFSET = 18;

// Roboterposition und Ziel
int startX = 0, startY = 0;
int goalX = 10, goalY = 10;

// Gitter: begehbar und Pfadmarkierung
bool walkable[GRID_SIZE][GRID_SIZE];
bool isPath[GRID_SIZE][GRID_SIZE] = { false };

// mm -> Gitter-Koordinate
int toGridCoord(double mm) {
  return static_cast<int>(round(mm / 50.0));
}

// GPS -> Startposition
void updateStartPositionFromGPS() {
  double gpsXmm = Gps1.xPosition(mm);
  double gpsYmm = Gps1.yPosition(mm);
  startX = toGridCoord(gpsXmm);
  startY = toGridCoord(gpsYmm);
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
  return abs(x1 - x2) + abs(y1 - y2);
}

void calculatePath() {
  static Node nodes[GRID_SIZE][GRID_SIZE];
  Node* openSet[GRID_SIZE * GRID_SIZE];
  bool closedSet[GRID_SIZE][GRID_SIZE] = { false };

  int openCount = 0;

  for (int y = -OFFSET; y <= OFFSET; y++) {
    for (int x = -OFFSET; x <= OFFSET; x++) {
      int rx = x + OFFSET;
      int ry = y + OFFSET;
      nodes[ry][rx] = {x, y, 9999, 9999, nullptr};
      isPath[ry][rx] = false;
    }
  }

  Node* start = &nodes[startY + OFFSET][startX + OFFSET];
  Node* goal  = &nodes[goalY + OFFSET][goalX + OFFSET];

  start->gCost = 0;
  start->hCost = heuristic(startX, startY, goalX, goalY);
  openSet[openCount++] = start;

  while (openCount > 0) {
    Node* current = findLowestFCost(openSet, openCount);

    int i = 0;
    for (; i < openCount; i++) {
      if (openSet[i] == current) break;
    }
    for (int j = i; j < openCount - 1; j++) {
      openSet[j] = openSet[j + 1];
    }
    openCount--;

    closedSet[current->y + OFFSET][current->x + OFFSET] = true;

    if (current == goal) {
      Node* n = goal->parent;
      while (n && n != start) {
        isPath[n->y + OFFSET][n->x + OFFSET] = true;
        n = n->parent;
      }
      return;
    }

    int dx[4] = { -1, 1, 0, 0 };
    int dy[4] = { 0, 0, -1, 1 };

    for (int d = 0; d < 4; d++) {
      int nx = current->x + dx[d];
      int ny = current->y + dy[d];

      if (nx < -OFFSET || nx > OFFSET || ny < -OFFSET || ny > OFFSET) continue;
      int rx = nx + OFFSET;
      int ry = ny + OFFSET;
      if (!walkable[ry][rx] || closedSet[ry][rx]) continue;

      int tentativeG = current->gCost + 1;
      Node* neighbor = &nodes[ry][rx];

      if (tentativeG < neighbor->gCost) {
        neighbor->parent = current;
        neighbor->gCost = tentativeG;
        neighbor->hCost = heuristic(nx, ny, goalX, goalY);

        bool inOpen = false;
        for (int i = 0; i < openCount; i++) {
          if (openSet[i] == neighbor) {
            inOpen = true;
            break;
          }
        }
        if (!inOpen) {
          openSet[openCount++] = neighbor;
        }
      }
    }
  }
}

void printGrid() {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);

  for (int y = -OFFSET; y <= OFFSET; y++) {
    for (int x = -OFFSET; x <= OFFSET; x++) {
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

int NAVI() {
  
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      walkable[y][x] = true;
      isPath[y][x] = false;
    }
  }

  for (int x = -5; x <= 5; x++) {
    walkable[10 + OFFSET][x + OFFSET] = false;
  }

  while (true) {
    updateStartPositionFromGPS();
    calculatePath();
    printGrid();
    wait(500, msec);
  }
}
