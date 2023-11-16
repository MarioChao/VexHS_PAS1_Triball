#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// Devices
controller Controller1(primary);
controller Controller2(partner);

motor LeftMotorA(PORT1, ratio18_1, true);
motor LeftMotorB(PORT2, ratio18_1, true);
motor LeftMotorC(PORT3, ratio18_1, true);
motor_group LeftMotors(LeftMotorA, LeftMotorB, LeftMotorC);
motor RightMotorA(PORT6, ratio18_1);
motor RightMotorB(PORT7, ratio18_1);
motor RightMotorC(PORT8, ratio18_1);
motor_group RightMotors(RightMotorA, RightMotorB, RightMotorC);
motor_group LeftRightMotors(LeftMotorA, LeftMotorB, LeftMotorC, RightMotorA, RightMotorB, RightMotorC);
motor LiftMotor1(PORT9, ratio36_1);
motor LiftMotor2(PORT20, ratio36_1);
motor_group LiftMotors(LiftMotor1, LiftMotor2);
motor FlywheelMotor(PORT10, ratio6_1, true);

pneumatics LeftWingPneumatic(Brain.ThreeWirePort.G);
pneumatics RightWingPneumatic(Brain.ThreeWirePort.H);
pneumatics AnchorPneumatic(Brain.ThreeWirePort.C);
pneumatics LiftClampPneumatic(Brain.ThreeWirePort.F);

inertial InertialSensor(PORT5);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}