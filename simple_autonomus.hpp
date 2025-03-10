/*bool isStuck() {
    double prevX = Gps1.xPosition(mm);
    double prevY = Gps1.yPosition(mm);
    vex::task::sleep(2000); // Wait 1 second to check for movement
    double newX = Gps1.xPosition(mm);
    double newY = Gps1.yPosition(mm);

    double distanceMoved = sqrt(pow(newX - prevX, 2) + pow(newY - prevY, 2));

    return distanceMoved < 100;  // Only return true if movement is <50mm
}
double getHeadingError(double targetHeading) {
    double headingError = targetHeading - GyroSensor.heading(degrees);

    // Normalize error to be between -180° and +180°
    if (headingError > 180) {
        headingError -= 360;
    } else if (headingError < -180) {
        headingError += 360;
    }
    
    return headingError;
}


/** 🚧 **Adjust for Robot Size/
const double ROBOT_WIDTH = 380; // 38cm
const double ROBOT_LENGTH = 380; // 36cm

// 🚧 **Wall & Obstacle Safety Zones*
const double WALL_THRESHOLD =50 + (ROBOT_WIDTH / 2);  // Stop before the wall
const double OBSTACLE_THRESHOLD = 100;//59 + (ROBOT_WIDTH / 2); // Stop before posts

// 📍 **Post Locations (Middle Square)**
struct Post {
    double x, y;
};
Post posts[4] = {
    {600, 600},   // Post 1
    {600, -600},  // Post 2
    {-600, 600},  // Post 3
    {-600, -600}  // Post 4
};

// 🚧 **Obstacle Avoidance Function (Smart Turn Towards Target)**

    
// Get average GPS position, handling errors
void getGPSPosition(double &x, double &y, double &heading) {
    double x1 = Gps1.xPosition(mm);
    double y1 = Gps1.yPosition(mm);
    double h1 = Gps1.heading(degrees);

    double x2 = Gps2.xPosition(mm);
    double y2 = Gps2.yPosition(mm);
    double h2 = Gps2.heading(degrees);

    // If the difference between GPS readings is too high, ignore the bad one
    if (fabs(x1 - x2) > 100 || fabs(y1 - y2) > 100) {  // If GPS1 & GPS2 differ by 100mm, ignore bad reading
        if (fabs(x1 - x2) > fabs(y1 - y2)) {  // If X is more off, use Y
            x = (fabs(x1) < fabs(x2)) ? x1 : x2;
            y = (y1 + y2) / 2;
        } else {  // If Y is more off, use X
            x = (x1 + x2) / 2;
            y = (fabs(y1) < fabs(y2)) ? y1 : y2;
        }
        heading = (h1 + h2) / 2;
    } else {
        // Use average if both are valid
        x = (x1 + x2) / 2;
        y = (y1 + y2) / 2;
        heading = (h1 + h2) / 2;
    }
}
void avoidObstacle(double targetX, double targetY) {
    Brain.Screen.print("🚧 Obstacle detected! Checking movement...");
    if (!isStuck()) {
        Brain.Screen.print("✅ Not stuck, continue driving.");
        return;  // Don't avoid if we're still moving!
    }

    Brain.Screen.print("⚠️ Confirmed obstacle! Avoiding...");

    double prevX, prevY, prevHeading;
    getGPSPosition(pre, prevY, prevHeading);
    
    vex::task::sleep(500);  // Wait 1 second to check for movement

    double newX, newY, newHeading;
    getGPSPosition(newX, newY, newHeading);

    double distanceMoved = sqrt(pow(newX - prevX, 2) + pow(newY - prevY, 2));

     //If the bot has moved at least 50mm, it's NOT stuck! No need to avoid.
    if (distanceMoved > 50) {
        Brain.Screen.print("✅ False alarm, not stuck.");
        return;
    }

    
 FullDrivetrain.spin(reverse,20,percent);
    // **Step 2: FULL STOP**
    vex::task::sleep(300);
    FullDrivetrain.stop();

    // **Step 3: Back Up**
    
    // **Step 4: Rotate 90° Toward the Target**
    double targetAngle = atan2(targetY - , targetX - Xpos) * 180.0 / M_PI;

    double leftTurn = newHeading + 90;
    double rightTurn = newHeading - 90;

   

    // Pick the turn that moves the bot closer to the target
    double leftError = fabs(targetAngle - leftTurn);
    double rightError = fabs(targetAngle - rightTurn);
    double bestTurn = (leftError < rightError) ? leftTurn : rightTurn;

    // Rotate bot until it faces the new heading
    while (fabs(newHeading - bestTurn) > 5) {
        LeftDrivetrain.spin(forward, 5, percent);
        RightDrivetrain.spin(reverse, 5, percent);
        vex::task::sleep(50);
    }
    FullDrivetrain.stop();
}

// 🚀 **Drive to GPS Coordinate & Avoid Obstacles**
void driveToCoordinate(double targetX, double targetY, double maxSpeed = 30) {
    double kP_turn = 0.4;
    double kP_speed = 0.6;
    double tolerance = 50;  // Larger bot needs a slightly bigger stop zone

    while (true) {
        // Get current position
        double X, Y, Heading;
        getGPSPosition(X, Y, Heading);

        // If near walls, back up and turn
        if (X < -900 + WALL_THRESHOLD || X > 900 - WALL_THRESHOLD ||
            Y < -900 + WALL_THRESHOLD || Y > 900 - WALL_THRESHOLD) {
            avoidObstacle(targetX, targetY);
            continue;
        }

        // Calculate distance to target
        double deltaX = targetX - X;
        double deltaY = targetY - Y;
        double distance = sqrt(deltaX * deltaX + deltaY * deltaY);

        // Stop if within tolerance
        if (distance < tolerance) {
            FullDrivetrain.stop();
            break;
        }

        // Calculate target heading
        double targetHeading = atan2(deltaY, deltaX) * 180.0 / M_PI;
        double headingError = getHeadingError(targetHeading);

        // Adjust movement based on heading error
        double turnPower = std::min(std::max(headingError * kP_turn, -15.0), 15.0);
        double driveSpeed = std::min(std::max(distance * kP_speed, 15.0), maxSpeed);

        // Slow down near the target
        if (distance < 150) {
            driveSpeed = std::min(driveSpeed, 20.0);
        }
// **🚨 Obstacle Detection: Check Posts**
        bool obstacleDetected = false;
        for (int i = 0; i < 4; i++) {
            double postX = posts[i].x;
            double postY = posts[i].y;
            double postDistance = sqrt(pow(X - postX, 2) + pow(Y - postY, 2));

            if (postDistance < OBSTACLE_THRESHOLD) {
                avoidObstacle(targetX,targetY);
                obstacleDetected = true;
                break;
            }
        }

        if (obstacleDetected) continue;  // Restart loop with new heading

        // 🚀 **Movement Logic**
       
        // Reduce speed near target
        if (distance < 150) {
            driveSpeed = std::min(driveSpeed, 20.0);
        }
        // Apply movement
        LeftDrivetrain.spin(forward, driveSpeed - turnPower, percent);
        RightDrivetrain.spin(forward, driveSpeed + turnPower, percent);

        vex::task::sleep(50);
    }
}        

        
    
    


// Main autonomous function



void turnXDegrees(double Angle) {
    GyroSensor.calibrate();  // Ensure the gyroscope is alibrated

    // Wait for the gyroscope to be ready
    while (GyroSensor.isCalibrating()) {
        vex::task::sleep(100);
    }

    // Reset the gyro angle to 0
   GyroSensor.resetRotation();

    while (GyroSensor.rotation() <  Angle) {
        LeftDrivetrain.spin(forward,20,percent);
        RightDrivetrain.spin(reverse,20,percent);
    }

    // Stop the motors once the turn is complete
    LeftDrivetrain.stop();
    RightDrivetrain.stop();
    

}  
