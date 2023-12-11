#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// Devices
controller Controller1(primary);
controller Controller2(partner);

motor LeftMotorA(PORT18, ratio6_1, true);
motor LeftMotorB(PORT19, ratio6_1, true);
motor LeftMotorC(PORT20, ratio6_1, true);
motor_group LeftMotors(LeftMotorA, LeftMotorB, LeftMotorC);
motor RightMotorA(PORT15, ratio6_1);
motor RightMotorB(PORT16, ratio6_1);
motor RightMotorC(PORT17, ratio6_1);
motor_group RightMotors(RightMotorA, RightMotorB, RightMotorC);
motor_group LeftRightMotors(LeftMotorA, LeftMotorB, LeftMotorC, RightMotorA, RightMotorB, RightMotorC);
motor IntakeMotor(PORT13, ratio6_1);
motor FlywheelMotor(PORT14, ratio6_1);

pneumatics LeftWingPneumatic(Brain.ThreeWirePort.G);
pneumatics RightWingPneumatic(Brain.ThreeWirePort.H);
pneumatics AnchorPneumatic(Brain.ThreeWirePort.C);
pneumatics LiftPneumatic1(Brain.ThreeWirePort.B);
pneumatics LiftPneumatic2(Brain.ThreeWirePort.F);

inertial InertialSensor(PORT12);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}