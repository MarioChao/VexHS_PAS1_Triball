#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// Devices
controller Controller1(primary);
controller Controller2(partner);

motor LeftMotorA(PORT11, ratio6_1, true);
motor LeftMotorB(PORT13, ratio6_1, true);
motor LeftMotorC(PORT14, ratio6_1, true);
motor RightMotorA(PORT20, ratio6_1);
motor RightMotorB(PORT19, ratio6_1);
motor RightMotorC(PORT18, ratio6_1);
motor_group LeftMotors(LeftMotorA, LeftMotorB, LeftMotorC);
motor_group RightMotors(RightMotorA, RightMotorB, RightMotorC);
motor_group LeftRightMotors(LeftMotorA, LeftMotorB, LeftMotorC, RightMotorA, RightMotorB, RightMotorC);
motor PuncherMotorA(PORT1, ratio18_1, true);
motor PuncherMotorB(PORT17, ratio18_1);
motor_group PuncherMotors(PuncherMotorA, PuncherMotorB);

pneumatics FrontWingsPneumatic(Brain.ThreeWirePort.D);
pneumatics LeftWingPneumatic(Brain.ThreeWirePort.B);
pneumatics RightWingPneumatic(Brain.ThreeWirePort.C);
pneumatics LiftPneumatic(Brain.ThreeWirePort.A);

encoder LookEncoder(Brain.ThreeWirePort.G);

inertial InertialSensor(PORT10);
distance DistanceSensor(PORT2);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}