#include "preauton.h"
#include "Graphics/BrainScreen.h"
#include "Mechanics/botLift.h"
#include "Utilities/debugFunctions.h"
#include "Videos/brainVideos.h"
#include "main.h"

namespace {
    void initComponents();
    void bufferScreen();
    
    bool initComponentFinished = false;
}


void preautonControllerThread() {
    debug::printOnController("No auton selected!");
    // eeecegg
    Controller1.rumble(".--.-- -");
}
void runPreauton() {
    // Buffer screen task
    task bufferTask([] () -> int {
        bufferScreen();
        return 1;
    });
    // Initialize components
    initComponents();
}
bool isPreautonFinished() {
    return initComponentFinished;
}

namespace {
    void initComponents() {
        initComponentFinished = false;

        // Calibrate inertial sensor
        InertialSensor.startCalibration();
        task::sleep(30);
        while (InertialSensor.isCalibrating()) {
            task::sleep(30);
        }
        printf("Calibrated!\n");
        // Pneumatic state
        setLiftState(true);
        
        initComponentFinished = true;
    }

    void bufferScreen() {
        // White / buffer screen
        Brain.Screen.clearScreen(color::white);
        Brain.Screen.setPenColor(color::black);
        Brain.Screen.setFillColor(color::white);
        timer bufferTimer;
        bufferTimer.reset();
        // printf("Competition: %d, Enabled: %d\n", Competition.isAutonomous(), Competition.isEnabled());
        while (!Competition.isAutonomous() && !Competition.isEnabled() && bufferTimer.value() <= 1) {
            Brain.Screen.printAt(240, 120, "%5.2f sec", bufferTimer.value());
            task::sleep(20);
        }
        // Shrinking circle
        Brain.Screen.setFillColor(color::transparent);
        Brain.Screen.setPenWidth(1);
        Brain.Screen.setPenColor(color::black);
        double prevR, newR;
        prevR = -1;
        for (double i = 0; ; i += 0.03) {
            if (i > 1) i = 1;
            newR = 275 - pow(i, 2) * 275;
            if (prevR == -1) prevR = newR + 1;
            // Draw ring
            Brain.Screen.setPenWidth(2 * fabs(newR - prevR));
            Brain.Screen.drawCircle(240, 120, newR);
            prevR = newR;
            task::sleep(20);
            // Exit when i is 1
            if (fabs(i - 1) <= 1e-8) break;
        }
        // Brain Screen
        task brainScreen([] () -> int {
            brainScreenThread();
            return 1;
        });
        task brainVideos([] () -> int {
            brainVideosThread();
            return 1;
        });
    }
}