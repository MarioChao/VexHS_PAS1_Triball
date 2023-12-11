#include "Utilities/robotInfo.h"
#include "main.h"

namespace botinfo {
    // Robot info
    double robotLengthIn = 23 * 0.500;
    double halfRobotLengthIn = robotLengthIn / 2;

    double driveWheelRadiusIn = 2.75 / 2;
    double driveWheelCircumIn = 2 * M_PI * driveWheelRadiusIn;
    double driveWheelMotorGearRatio = (48.0 / 36.0);

    // Velocity from pct to rev/s
    // 100% = 600/60 rev/s
    // 100.0 * 60.0 / 600.0 (%s/rev) = 1
    double driveMotorRevToPercentSecFactor = 100.0 * 60.0 / 600.0;
}