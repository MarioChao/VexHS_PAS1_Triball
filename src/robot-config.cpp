#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// Devices
controller Controller1(primary);
controller Controller2(partner);

motor LeftMotorA(PORT18, ratio6_1);
motor LeftMotorB(PORT19, ratio6_1);
motor LeftMotorC(PORT17, ratio6_1);
motor RightMotorA(PORT12, ratio6_1, true);
motor RightMotorB(PORT15, ratio6_1, true);
motor RightMotorC(PORT14, ratio6_1, true);
motor_group LeftMotors(LeftMotorA, LeftMotorB, LeftMotorC);
motor_group RightMotors(RightMotorA, RightMotorB, RightMotorC);
motor_group LeftRightMotors(LeftMotorA, LeftMotorB, LeftMotorC, RightMotorA, RightMotorB, RightMotorC);
motor PuncherMotorA(PORT1, ratio18_1);
motor PuncherMotorB(PORT2, ratio18_1, true);
motor_group PuncherMotors(PuncherMotorA, PuncherMotorB);

pneumatics LeftWingPneumatic(Brain.ThreeWirePort.G);
pneumatics RightWingPneumatic(Brain.ThreeWirePort.H);
pneumatics LiftPneumatic(Brain.ThreeWirePort.A);

inertial InertialSensor(PORT16);
distance DistanceSensor(PORT3);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}