#include "vex.h"

using namespace vex;

void rampUp(motor_group& drive, double finalSpeed)
{
    double interval = 20;
    double rampTime = 0.5;
    double startSpeed = 20;
    double currentSpeed = startSpeed;
    double step = (finalSpeed - startSpeed) / (rampTime * 1000 / interval);

    while (currentSpeed < finalSpeed)
    {
        drive.spin(forward, currentSpeed, percent);
        currentSpeed += step;
        task::sleep(interval);
    }

    drive.spin(forward, finalSpeed, percent);
}

