#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// Devices
controller Controller1(primary);
controller Controller2(partner);

motor LeftMotorA(PORT17, ratio6_1);
motor LeftMotorB(PORT18, ratio6_1);
motor LeftMotorC(PORT19, ratio6_1);
motor LeftMotorD(PORT20, ratio6_1);
motor RightMotorA(PORT11, ratio6_1, true);
motor RightMotorB(PORT12, ratio6_1, true);
motor RightMotorC(PORT14, ratio6_1, true);
motor RightMotorD(PORT15, ratio6_1, true);
motor_group LeftMotors(LeftMotorA, LeftMotorB, LeftMotorC, LeftMotorD);
motor_group RightMotors(RightMotorA, RightMotorB, RightMotorC, RightMotorD);
motor_group LeftRightMotors(LeftMotorA, LeftMotorB, LeftMotorC, LeftMotorD, RightMotorA, RightMotorB, RightMotorC, RightMotorD);

pneumatics IntakePneumatic(Brain.ThreeWirePort.A);
pneumatics LeftWingPneumatic(Brain.ThreeWirePort.B);
pneumatics RightWingPneumatic(Brain.ThreeWirePort.C);
pneumatics LiftPneumatic1(Brain.ThreeWirePort.E);
pneumatics LiftPneumatic2(Brain.ThreeWirePort.F);

inertial InertialSensor(PORT13);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}