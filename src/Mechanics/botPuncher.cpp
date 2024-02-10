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

    int punchedCount = 0;

    bool runPuncherDebounce = false;
    bool canSpinPuncher = false;

    bool isPuncherResetted = false;

    bool disablePuncher = true;
}

void resetPuncherCall() {
    if (disablePuncher) {
        return;
    }
    
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
    if (disablePuncher) {
        return;
    }

    // Set puncher brake types
    PuncherMotors.setStopping(hold);

    // Puncher loop
    punchedCount = 0;
    while (true) {
        resolvePuncher();
        task::sleep(20);
    }
}

int getPunchedCount() {
    return punchedCount;
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

        // Validate puncher
        if (!PuncherMotorA.installed()) {
            return;
        }

        // Check if an object is within detection distance
        if (DistanceSensor.objectDistance(mm) <= ballDetectionDistanceMm) {
            // Punch once
            runPuncherOnce();
            punchedCount++;
            printf("Punched: %d\n", punchedCount);
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
        // TODO: tune the pid to make the puncher faster
        PIDControl rotateTargetAnglePid(0.5, 0, 0.03, 7.0, 5.0);
        timer runTimeout;

        // Spin puncher to position
        // while (!rotateTargetAnglePid.isSettled() && runTimeout.value() < 0.5) {
        //     // Get error
        //     double currentMotorDegrees = PuncherMotors.position(deg);
        //     double error = targetMotorDegrees - currentMotorDegrees;

        //     // Get velocity from pid
        //     rotateTargetAnglePid.computeFromError(error);
        //     double spinVelocity = rotateTargetAnglePid.getValue();

        //     // Spin puncher
        //     spinPuncherVelocity(spinVelocity);
        //     task::sleep(10);
        // }

        // Spin puncher until position
        PuncherMotors.spin(fwd, 12, volt);
        while (PuncherMotors.position(deg) < targetMotorDegrees) {
            task::sleep(20);
        }

        // Stop puncher
        spinPuncherVelocity(0);
        printf("TAR: %.3f, DEG: %.3f\n", targetMotorDegrees, PuncherMotors.position(deg));
    }
    void spinPuncherVelocity(double velocityPct) {
        PuncherMotors.spin(fwd, velocityPct, pct);
    }
}