#pragma once

enum autonomousType {
    Left,
    Right,
    NearAWP,
    NearElim,
    FarAWP,
    FarElim,
    Skills,
    Test,
    None
};

/// @brief Set the type of autonomous to run at the start of the match.
/// @param allianceId A number representing the alliance (Red: 1, Blue: 2, Neutral: 0).
/// @param autonType The type of autonomus to run.
void setAutonRunType(int allianceId, autonomousType autonType);

/// @brief Run the autonomous set by setAutonRunType().
void runAutonomous();