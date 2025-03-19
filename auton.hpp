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
void hardauton(void){

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
void AIMODE(void){

}
void autonomous() {
    // Example: grid-based navigation to target (900, 80).
    
    hardauton();
    
    // Additional functions (e.g., stakerclose, intake routines) can follow.
}

   