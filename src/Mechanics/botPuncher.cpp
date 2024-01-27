#include "Mechanics/botPuncher.h"
#include "Utilities/pidControl.h"
#include "main.h"

namespace {
    void resetPuncher();
    void resolvePuncher();
    void runPuncherOnce();
    void spinPuncherToPosition(double targetMotorDegrees);
    void spinPuncherVelocity(double velocityPct);

    double resetPuncherMotorGearTeeths = 8.0;
    double motorPuncherGearTotalTeeths = 12.0;
    double ballDetectionDistanceMm = 40;
    
    int currentPuncherRevolution = 0;

    bool runPuncherDebounce = false;
    bool canSpinPuncher = false;

    bool isPuncherResetted = false;
}

void resetPuncherCall() {
    // Validate isPuncherResetted
    if (isPuncherResetted) {
        return;
    }

    // Reset puncher task
    task resetPuncherTask([] () -> int {
        // Reset puncher and set canSpinPuncher
        canSpinPuncher = false;
        resetPuncher();
        canSpinPuncher = true;
        isPuncherResetted = true;

        return 1;
    });
}

void puncherThread() {
    // Set puncher brake types
    PuncherMotors.setStopping(hold);

    // Puncher loop
    while (true) {
        resolvePuncher();
        task::sleep(20);
    }
}

namespace {
    void resetPuncher() {
        // Get target rotation
        PuncherMotors.setPosition(0, deg);
        double resetPuncherTargetMotorDegrees = (resetPuncherMotorGearTeeths / motorPuncherGearTotalTeeths) * 360.0;

        // Spin puncher
        spinPuncherToPosition(resetPuncherTargetMotorDegrees);

        // Reset puncher
        PuncherMotors.setPosition(0, deg);
        currentPuncherRevolution = 0;
    }
    void resolvePuncher() {
        // Validate canSpinPuncher
        if (!canSpinPuncher) {
            return;
        }

        // Check if an object is within detection distance
        if (DistanceSensor.objectDistance(mm) <= ballDetectionDistanceMm) {
            // Punch once
            runPuncherOnce();
        }
    }
    void runPuncherOnce() {
        if (!runPuncherDebounce) {
            runPuncherDebounce = true;

            // Get target rotation
            double targetPuncherMotorDegrees = (currentPuncherRevolution + 1) * 360.0;
            currentPuncherRevolution++;

            // Spin puncher
            spinPuncherToPosition(targetPuncherMotorDegrees);

            runPuncherDebounce = false;
        }
    }
    void spinPuncherToPosition(double targetMotorDegrees) {
        // Set up pid
        PIDControl rotateTargetAnglePid(0.42, 0, 0, 7.0, 5.0);
        timer runTimeout;

        // Spin puncher to position
        while (!rotateTargetAnglePid.isSettled() && runTimeout.value() < 0.6) {
            // Get error
            double currentMotorDegrees = PuncherMotors.position(deg);
            double error = targetMotorDegrees - currentMotorDegrees;

            // Get velocity from pid
            rotateTargetAnglePid.computeFromError(error);
            double spinVelocity = rotateTargetAnglePid.getValue();

            // Spin puncher
            spinPuncherVelocity(spinVelocity);
            task::sleep(10);
        }

        // Stop puncher
        spinPuncherVelocity(0);
        printf("TAR: %.3f, DEG: %.3f\n", targetMotorDegrees, PuncherMotors.position(deg));
    }
    void spinPuncherVelocity(double velocityPct) {
        PuncherMotors.spin(fwd, velocityPct, pct);
    }
}