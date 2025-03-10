#include "vex.h"
#include <math.h>
#include <algorithm>
#include <fstream>


void EVRYTHING_IS_FUCKED(void){
    vex::task::sleep(1000);
    DriveReverse(1200);
    stakerclose();
    vex::task::sleep(2*1000);
    turnXDegrees(250);
    DriveReverse(1000);
    vex::task::sleep(2*1000);
    DriveForward(500);
    vex::task::sleep(2*1000);
    inntake(); 
}
/*void TesT(void){
    

    const int GRID_SIZE = 20; // Define grid resolution (smaller = more precise, but slower)
    const double CELL_SIZE = 180; // Each grid cell is 180mm x 180mm
    const double FIELD_SIZE = 3600;
    const double WALL_THRESHOLD = 100;
    const double OBSTACLE_THRESHOLD = 200;
    const double ROBOT_WIDTH = 380;
    
    // 📍 Posts in the middle of the field
    struct Post {
        int x, y;
    };
    std::vector<Post> obstacles = {
        {598 / CELL_SIZE, 598 / CELL_SIZE}, 
        {598 / CELL_SIZE, -598 / CELL_SIZE},
        {-598 / CELL_SIZE, 598 / CELL_SIZE},
        {-598 / CELL_SIZE, -598 / CELL_SIZE}
    };
    
    // 🚀 **A* Node Structure**
    struct Node {
        int x, y;
        double g, h, f;
        Node* parent;
    
        Node(int x, int y, double g, double h, Node* parent) 
            : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}
    };
    
    // 🚀 **Heuristic Function (Euclidean Distance)**
    double heuristic(int x1, int y1, int x2, int y2) {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }
    
    // 🚀 **A* Pathfinding Algorithm**
    std::vector<Node> aStar(int startX, int startY, int goalX, int goalY) {
        std::vector<std::vector<bool>> visited(GRID_SIZE, std::vector<bool>(GRID_SIZE, false));
        std::priority_queue<Node, std::vector<Node>, 
            [](const Node& a, const Node& b) { return a.f > b.f; }> openSet;
        
        openSet.push(Node(startX, startY, 0, heuristic(startX, startY, goalX, goalY), nullptr));
    
        while (!openSet.empty()) {
            Node current = openSet.top();
            openSet.pop();
    
            if (current.x == goalX && current.y == goalY) {
                std::vector<Node> path;
                while (current.parent) {
                    path.push_back(current);
                    current = *current.parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
    
            visited[current.x][current.y] = true;
            
            // Movement directions (Up, Down, Left, Right)
            std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    
            for (auto [dx, dy] : directions) {
                int newX = current.x + dx;
                int newY = current.y + dy;
                if (newX >= 0 && newX < GRID_SIZE && newY >= 0 && newY < GRID_SIZE && !visited[newX][newY]) {
                    bool isObstacle = false;
                    for (auto post : obstacles) {
                        if (post.x == newX && post.y == newY) {
                            isObstacle = true;
                            break;
                        }
                    }
                    if (!isObstacle) {
                        openSet.push(Node(newX, newY, current.g + 1, heuristic(newX, newY, goalX, goalY), &current));
                    }
                }
            }
        }
        return {}; // No path found
    }
    
    // 🚀 **Function: Drive Along A* Path**
    void followPath(std::vector<Node> path) {
        for (const auto& node : path) {
            double targetX = node.x * CELL_SIZE - FIELD_SIZE / 2;
            double targetY = node.y * CELL_SIZE - FIELD_SIZE / 2;
            driveToCoordinate(targetX, targetY);
        }
    }
    
    // 🚀 **Main Autonomous Routine with A***
    void autonom(void) {
        double startX, startY, heading;
        getGPSPosition(startX, startY, heading);
        
        int gridStartX = (startX + FIELD_SIZE / 2) / CELL_SIZE;
        int gridStartY = (startY + FIELD_SIZE / 2) / CELL_SIZE;
        int gridGoalX = (900 + FIELD_SIZE / 2) / CELL_SIZE;
        int gridGoalY = (80 + FIELD_SIZE / 2) / CELL_SIZE;
    
        std::vector<Node> path = aStar(gridStartX, gridStartY, gridGoalX, gridGoalY);
        followPath(path);
    }
    }
-------------------------------------------------------------------------------------------------------------------------------------
using namespace vex;
const double FIELD_SIZE = 3600;
const double WALL_THRESHOLD = 100;  // Stop before walls
const double OBSTACLE_THRESHOLD = 150;  // Avoid posts
const double ROBOT_WIDTH = 380;

// Path Logging
std::ofstream logFile("/usd/navigation_log.txt");

// Posts in the middle of the field
struct Post {
    double x, y;
};
Post posts[4] = {
    {598.678 ,598.678 }, {598.678 , -598.678}, {-598.678 , 598.678 }, {-598.678 , -598.678 }
};

void getGPSPosition(double &x, double &y, double &heading) {
    double x1 = Gps1.xPosition(mm), y1 = Gps1.yPosition(mm), h1 = Gps1.heading(degrees);
    double x2 = Gps2.xPosition(mm), y2 = Gps2.yPosition(mm), h2 = Gps2.heading(degrees);

    if (fabs(x1 - x2) > 100 || fabs(y1 - y2) > 100) {
        x = (fabs(x1) < fabs(x2)) ? x1 : x2;
        y = (fabs(y1) < fabs(y2)) ? y1 : y2;
    } else {
        x = (x1 + x2) / 2;
        y = (y1 + y2) / 2;
    }
    heading = Gps1.heading(degrees);
}

double getHeadingError(double targetHeading) {
    double headingError = targetHeading - Gps1.heading(degrees);
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;
    return headingError;
}

void turnToHeading(double targetHeading) {
    while (fabs(getHeadingError(targetHeading)) > 5) {
        double headingError = getHeadingError(targetHeading);
        double turnSpeed = std::max(std::min(headingError * 0.5, 30.0), -30.0);
        LeftDrivetrain.spin(forward, turnSpeed, percent);
        RightDrivetrain.spin(reverse, turnSpeed, percent);
        vex::task::sleep(50);
    }
    FullDrivetrain.stop();
}

void driveToCoordinate(double targetX, double targetY, double maxSpeed = 20) {
    double kP_turn = 0.4, kP_speed = 0.6, tolerance = 50;

    while (true) {
        double X, Y, Heading;
        getGPSPosition(X, Y, Heading);
        double deltaX = targetX - X, deltaY = targetY - Y;
        double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
        if (distance < tolerance) { FullDrivetrain.stop(); break; }

        double targetHeading = atan2(deltaY, deltaX) * 180.0 / M_PI;
        if (fabs(getHeadingError(targetHeading)) > 30) {
            turnToHeading(targetHeading);
        }

        double headingError = getHeadingError(targetHeading);
        double turnPower = std::max(std::min(headingError * kP_turn, 10.0), -10.0);
        double driveSpeed = std::max(std::min(distance * kP_speed, maxSpeed), 20.0);
        if (distance < 150) driveSpeed = std::min(driveSpeed, 20.0);

        LeftDrivetrain.spin(forward, driveSpeed - turnPower, percent);
        RightDrivetrain.spin(forward, driveSpeed + turnPower, percent);
        vex::task::sleep(50);
    }
}

void autonomous(void) {
    driveToCoordinate(900, 80);
    driveToCoordinate(-900, -500);
}