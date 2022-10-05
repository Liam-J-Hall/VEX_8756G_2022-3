#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor MotorA = motor(PORT1, ratio18_1, false);
motor MotorB = motor(PORT2, ratio18_1, false);
motor MotorC = motor(PORT3, ratio18_1, false);
motor MotorD = motor(PORT4, ratio18_1, false);
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
bool DrivetrainNeedsToBeStopped_Controller1 = true;

/*
int upDown = Controller1.Axis3.position();
int leftRight = Controller1.Axis4.position();
int botRot = Controller1.Axis1.position();
*/

/*
int MotorAVel = Controller1.Axis3.position() + Controller1.Axis4.position() + Controller1.Axis1.position();
int MotorBVel = -Controller1.Axis3.position() + Controller1.Axis4.position() + Controller1.Axis1.position();
int MotorCVel = -Controller1.Axis3.position() - Controller1.Axis4.position() + Controller1.Axis1.position();
int MotorDVel = Controller1.Axis3.position() - Controller1.Axis4.position() + Controller1.Axis1.position();
*/

void move(){
  int MotorAVel = Controller1.Axis3.position() + Controller1.Axis4.position() + Controller1.Axis1.position();
  int MotorBVel = -Controller1.Axis3.position() + Controller1.Axis4.position() + Controller1.Axis1.position();
  int MotorCVel = -Controller1.Axis3.position() - Controller1.Axis4.position() + Controller1.Axis1.position();
  int MotorDVel = Controller1.Axis3.position() - Controller1.Axis4.position() + Controller1.Axis1.position();

  MotorA.setVelocity(MotorAVel, percent);
  MotorB.setVelocity(MotorBVel, percent);
  MotorC.setVelocity(MotorCVel, percent);
  MotorD.setVelocity(MotorDVel, percent);

  MotorA.spin(forward);
  MotorB.spin(forward);
  MotorC.spin(forward);
  MotorD.spin(forward);
}

/*
[motor].setVelocity(drivetrainLeftSideSpeed, percent);
MotorA.spin(forward);

motor[A ] = +Ch3 +Ch4 +Ch1;
motor[B ] = -Ch3 +Ch4 +Ch1;
motor[C ] = -Ch3 -Ch4 +Ch1;
motor[D] = +Ch3 -Ch4 +Ch1;
*/

/*
***CONTROLS/MOTORS***
AXIS 3 - MOVE VERTICALLY // 1-2
AXIS 4 - MOVE HORIZONTALLY // 3-4
AXIS 1 - ROTATE ROBOT // 1-4
R1 - SHOOT DISC // 5
A - SPIN ROLLER // 6
B - POWER FLY WHEEL // 7-8
X - EXPAND ROBOT // 9
UP ARROW - TURN ON/OFF INTAKE MOTOR // 10
*/

int rc_auto_loop_function_Controller1() {
 // process the controller input every 20 milliseconds
 // update the motors based on the input values

 while(true) {
   if(RemoteControlCodeEnabled) {
     // calculate the drivetrain motor velocities from the controller joystick axies
     move();
   }
 }
}

/*
 *
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */

void vexcodeInit( void ) {
  // nothing to initialize
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);

}