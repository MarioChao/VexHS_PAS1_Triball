#include "Utilities/robotInfo.h"
#include "main.h"

namespace botinfo {
    // Robot info
    double robotLengthIn = 25.5 * (1.0 / 2.0); // 1 inch / 2 holes
    double halfRobotLengthIn = robotLengthIn / 2;

    double driveWheelRadiusIn = 3.00 / 2;
    double driveWheelCircumIn = 2 * M_PI * driveWheelRadiusIn;
    double driveWheelMotorGearRatio = (60.0 / 36.0);

    // Velocity from pct to rev/s
    // 100% = 600/60 rev/s
    // 100.0 * 60.0 / 600.0 (%s/rev) = 1
    double driveMotorRevToPercentSecFactor = 100.0 * 60.0 / 600.0;

    double trackingLookWheelRadiusIn = 2.00 / 2;
    double trackingLookWheelCircumIn = 2 * M_1_PI * trackingLookWheelRadiusIn;
    double trackingLookWheelEncoderGearRatio = 1.0;
}