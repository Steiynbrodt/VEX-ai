#include "vex.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <queue>
#include <fstream>
#include <limits>
using namespace vex;

const double TOLERANCE = 50.0;         // mm: distance to consider target reached
const double MLINE_THRESHOLD = 50.0;
const double FIELD_SIZE = 3600;
const double WALL_THRESHOLD = 200.00;
const double OBSTACLE_THRESHOLD = 50.00;
const double ROBOT_WIDTH = 380;

// Field grid parameters
const double MIN_FIELD = -1800.0;
const double MAX_FIELD = 1800.0;
const double CELL_SIZE = 50.0;        // Fine grid resolution: 50 mm per cell
const int GRID_SIZE = (int)((MAX_FIELD - MIN_FIELD) / CELL_SIZE); // e.g., 72 cells

volatile bool robotStuck = false;  // Global flag for stuck status

//------------------- SENSOR & DRIVE FUNCTIONS -------------------//


// Returns the heading error between target and current heading (from Gps1)
double getHeadingError(double targetHeading) {
    double headingError = targetHeading - Gps1.heading(degrees);
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;
    return headingError;
}

// Manual turn function (using Gps1) to turn until within 5° of target heading
void turnToHeadingmanual(double targetHeading) {
    while (fabs(getHeadingError(targetHeading)) > 5) {
        double headingError = getHeadingError(targetHeading);
        double turnSpeed = std::max(std::min(headingError * 0.5, 30.0), -30.0);
        LeftDrivetrain.spin(forward, turnSpeed, percent);
        RightDrivetrain.spin(reverse, turnSpeed, percent);
        task::sleep(50);
    }
    FullDrivetrain.stop();
}

// Posts in the middle of the field
struct Post {
    double x, y;
};
Post postsArr[4] = {
    {598.678, 598.678}, {598.678, -598.678}, {-598.678, 598.678}, {-598.678, -598.678}
};

// Get GPS Position & Heading (combining two sensors)
void getGPSPosition(double &x, double &y, double &heading) {
    double x1 = Gps1.xPosition(mm), y1 = Gps1.yPosition(mm);
    double h1 = Gps1.heading(degrees);
    double x2 = Gps2.xPosition(mm), y2 = Gps2.yPosition(mm);
    
    if (fabs(x1 - x2) > 100 || fabs(y1 - y2) > 100) {
        x = (fabs(x1) < fabs(x2)) ? x1 : x2;
        y = (fabs(y1) < fabs(y2)) ? y1 : y2;
    } else {
        x = (x1 + x2) / 2;
        y = (y1 + y2) / 2;
    }
    heading = h1;
}
bool isStuck() {
    double prevX, prevY, newX, newY, heading;
    getGPSPosition(prevX, prevY, heading);
    task::sleep(1500);
    getGPSPosition(newX, newY, heading);
    
    bool stuck = sqrt(pow(newX - prevX, 2) + pow(newY - prevY, 2)) < 150;
    return stuck;
}
void avoidObstacle() {
    Brain.Screen.print("🚧 Avoiding Obstacle...");
    FullDrivetrain.spin(reverse, 20, percent);
    task::sleep(1200);
    FullDrivetrain.stop();
    
    LeftDrivetrain.spin(forward, 20, percent);
    RightDrivetrain.spin(reverse, 20, percent);
    task::sleep(1700);
    FullDrivetrain.stop();
}

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

//------------------- HARDWARE FUNCTIONS -------------------//

// Hard-coded function to turn the stake mechanism.
void stakerclose(void) {
  vex::task::sleep(400);
  Pneumatic1.set(!Pneumatic1.value());
  
  
  
}

// Hard-coded turning: turn the robot until a designated heading is reached.
void turnXDegrees(double Angle1){
    INS.calibrate();
    while (INS.isCalibrating()) {
        vex::task::sleep(700);
    }
    vex::task::sleep(500);
    while (INS.heading(degrees) < Angle1){
        LeftDrivetrain.spin(forward, 20, percent);
        RightDrivetrain.spin(reverse, 20, percent);
    }
    LeftDrivetrain.stop();
    RightDrivetrain.stop();
}

void spinfor(int TT){
    LeftDrivetrain.spin(forward, 30, percent);
    RightDrivetrain.spin(reverse, 30, percent);
    task::sleep(TT);
    FullDrivetrain.stop(brake);

}
void spin45(){
  LeftDrivetrain.spin(forward, 30, percent);
    RightDrivetrain.spin(reverse, 30, percent);
    task::sleep(250);
    FullDrivetrain.stop(brake);
}

void spin90(){
  LeftDrivetrain.spin(forward, 30, percent);
  RightDrivetrain.spin(reverse, 30, percent);
  task::sleep(500);
  FullDrivetrain.stop(brake);
}
void spinforCC(int OO){
  LeftDrivetrain.spin(reverse, 30, percent);
  RightDrivetrain.spin(forward, 30, percent);
  task::sleep(OO);
  FullDrivetrain.stop(brake);

}
void spin45CC(){
LeftDrivetrain.spin(reverse, 30, percent);
  RightDrivetrain.spin(forward, 30, percent);
  task::sleep(250);
  FullDrivetrain.stop(brake);
}

void spin90CC(){
LeftDrivetrain.spin(reverse, 30, percent);
RightDrivetrain.spin(forward, 30, percent);
task::sleep(500);
FullDrivetrain.stop(brake);
}

void driveforwardfor(int II){
  FullDrivetrain.spin(forward,40,percent);
  task::sleep(II);
  FullDrivetrain.stop();
}

void drivebackwardfor(int IO){
  FullDrivetrain.spin(reverse,40,percent);
  task::sleep(IO);
  FullDrivetrain.stop();
}
//------------------- AUTONOMOUS ROUTINE -------------------//
void vrcauton(void){

}
void aiauton(void){

  driveforwardfor(440);inntake(2500);
spinfor(1090);
stakerclose();
drivebackwardfor(800);
stakerclose();
inntake(2000);
vex::task::sleep(500);
spinfor(1000);
driveforwardfor(1000);inntake(2500);
spinforCC(700);
driveforwardfor(1000);inntake(2500);
spinforCC(545);
driveforwardfor(2250);inntake(2500);
inntake(2500);
drivebackwardfor(300);
spinfor(1090);
drivebackwardfor(700);
stakerclose();
vex::task::sleep(1000);
driveforwardfor(300); 
stakerclose();



 



}
void autonomous() {
    // Example: grid-based navigation to target (900, 80).
    //gridBasedNavigate(900, 800);
    aiauton();
    
    // Additional functions (e.g., stakerclose, intake routines) can follow.
}

   