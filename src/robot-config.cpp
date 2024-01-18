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
motor LeftMotorD(PORT20, ratio6_1);
motor RightMotorA(PORT12, ratio6_1, true);
motor RightMotorB(PORT15, ratio6_1, true);
motor RightMotorC(PORT14, ratio6_1, true);
motor RightMotorD(PORT11, ratio6_1, true);
motor_group LeftMotors(LeftMotorA, LeftMotorB, LeftMotorC, LeftMotorD);
motor_group RightMotors(RightMotorA, RightMotorB, RightMotorC, RightMotorD);
motor_group LeftRightMotors(LeftMotorA, LeftMotorB, LeftMotorC, LeftMotorD, RightMotorA, RightMotorB, RightMotorC, RightMotorD);

pneumatics IntakePneumatic(Brain.ThreeWirePort.B);
pneumatics LeftWingPneumatic(Brain.ThreeWirePort.G);
pneumatics RightWingPneumatic(Brain.ThreeWirePort.H);
pneumatics LiftPneumatic(Brain.ThreeWirePort.A);

inertial InertialSensor(PORT16);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}