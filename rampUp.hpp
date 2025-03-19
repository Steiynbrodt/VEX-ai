#include "vex.h"

using namespace vex;

void rampUp( double finalSpeed)
{
    double interval = 20;
    double rampTime = 0.5;
    double startSpeed = 20;
    double currentSpeed = startSpeed;
    double step = (finalSpeed - startSpeed) / (rampTime * 1000 / interval);

    while (currentSpeed < finalSpeed)
    {
        FullDrivetrain.spin(forward, currentSpeed, percent);
        currentSpeed += step;
        task::sleep(interval);
    }

    FullDrivetrain.spin(forward, finalSpeed, percent);
}

