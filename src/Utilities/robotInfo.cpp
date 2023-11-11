#include "Utilities/robotInfo.h"
#include "main.h"

namespace botinfo {
    // Robot info
    double robotLengthIn = 21 * 0.500;
    double halfRobotLengthIn = robotLengthIn / 2;

    double driveWheelRadiusIn = 3.25 / 2;
    double driveWheelCircumIn = 2 * M_PI * driveWheelRadiusIn;
    double driveWheelMotorGearRatio = (30.0 / 60.0);

    // Velocity from pct to rev/s
    // 100% = 200/60 rev/s
    // 100.0 * 60.0 / 200.0 (%s/rev) = 1
    double driveMotorRevToPercentSecFactor = 100.0 * 60.0 / 200.0;
}