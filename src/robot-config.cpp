#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// Devices
controller Controller1(primary);
controller Controller2(partner);

motor LeftMotorA(PORT1, ratio6_1);
motor LeftMotorB(PORT2, ratio6_1);
motor LeftMotorC(PORT3, ratio6_1);
motor LeftMotorD(PORT4, ratio6_1);
motor RightMotorA(PORT7, ratio6_1, true);
motor RightMotorB(PORT8, ratio6_1, true);
motor RightMotorC(PORT9, ratio6_1, true);
motor RightMotorD(PORT10, ratio6_1, true);
motor_group LeftMotors(LeftMotorA, LeftMotorB, LeftMotorC, LeftMotorD);
motor_group RightMotors(RightMotorA, RightMotorB, RightMotorC, RightMotorD);
motor_group LeftRightMotors(LeftMotorA, LeftMotorB, LeftMotorC, LeftMotorD, RightMotorA, RightMotorB, RightMotorC, RightMotorD);

pneumatics IntakePneumatic(Brain.ThreeWirePort.A);
pneumatics LeftWingPneumatic(Brain.ThreeWirePort.B);
pneumatics RightWingPneumatic(Brain.ThreeWirePort.C);
pneumatics LiftPneumatic1(Brain.ThreeWirePort.E);
pneumatics LiftPneumatic2(Brain.ThreeWirePort.F);

inertial InertialSensor(PORT6);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}