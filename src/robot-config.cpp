#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// Devices
controller Controller1(primary);
controller Controller2(partner);

motor LeftMotorA(PORT19, ratio6_1, true);
motor LeftMotorB(PORT20, ratio6_1, true);
motor LeftMotorC(PORT17, ratio6_1);
motor LeftMotorD(PORT18, ratio6_1);
motor RightMotorA(PORT11, ratio6_1);
motor RightMotorB(PORT13, ratio6_1);
motor RightMotorC(PORT12, ratio6_1, true);
motor RightMotorD(PORT14, ratio6_1, true);
motor_group LeftMotors(LeftMotorA, LeftMotorB, LeftMotorC, LeftMotorD);
motor_group RightMotors(RightMotorA, RightMotorB, RightMotorC, RightMotorD);
motor_group LeftRightMotors(LeftMotorA, LeftMotorB, LeftMotorC, LeftMotorD, RightMotorA, RightMotorB, RightMotorC, RightMotorD);

pneumatics LeftWingPneumatic(Brain.ThreeWirePort.B);
pneumatics RightWingPneumatic(Brain.ThreeWirePort.C);
pneumatics LiftPneumatic1(Brain.ThreeWirePort.E);
pneumatics LiftPneumatic2(Brain.ThreeWirePort.F);

inertial InertialSensor(PORT10);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}