#pragma once

namespace auton {
    void setRotation(double rotation);
    void turnToAngle(double rotation, double rotateCenterOffsetIn = 0, double errorRange = 3, double runTimeout = 3);
    void turnToAngleVelocity(double rotation, double maxVelocityPct = 100.0, double rotateCenterOffsetIn = 0, double errorRange = 3, double runTimeout = 3);
    
    void driveDistanceTiles(double distanceTiles, double maxVelocityPct = 100, double errorRange = 0.05, double runTimeout = 3);
    void driveAndTurnDistanceTiles(double distanceTiles, double targetRotation, double rotationFactor = 1.0, double maxVelocityPct = 100, double errorRange = 0.05, double runTimeout = 3);
    void driveAndTurnDistanceTilesMotionProfile(double distanceTiles, double targetRotation, double maxVelocityPct = 100, double errorRange = 0.05, double runTimeout = 3);
    void driveAndTurnDistanceWithInches(double distanceInches, double targetRotation, double rotationFactor = 1.0, double maxVelocityPct = 100, double errorRange = 0.3, double runTimeout = 3);
    void driveAndTurnDistanceWithInchesMotionProfile(double distanceInches, double targetRotation, double maxVelocityPct = 100, double errorRange = 0.3, double runTimeout = 3);

    void setIntakeState(bool state);

    void setLiftPositionValue(double rotation);
    void setLiftToDegreeRotation(double rotation, double runTimeout = -1.0);
    void liftToDegreeTask();

    void setFlywheelSpeedRpm(double rpm);

    void setLeftWingState(bool state);
    void setRightWingState(bool state);
    void setWingsState(bool state);

    void setAnchorState(bool state);

    void setLiftClampState(bool state);
}