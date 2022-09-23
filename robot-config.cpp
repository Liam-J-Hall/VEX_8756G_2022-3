#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
bool DrivetrainNeedsToBeStopped_Controller1 = true;

int upDown = Controller1.Axis3.position();
int leftRight = Controller1.Axis4.position();
int botRot = Controller1.Axis1.position();

/*
***CONTROLS***

AXIS 3 - MOVE VERTICALLY
AXIS 4 - MOVE HORIZONTALLY

AXIS 1 - ROTATE ROBOT

R1 - SHOOT DISC
L1 - 

A - SPIN ROLLER
B - POWER FLY WHEEL
X - EXPAND ROBOT

UP ARROW - TURN ON/OFF INTAKE MOTOR




*/

int rc_auto_loop_function_Controller1() {
 // process the controller input every 20 milliseconds
 // update the motors based on the input values
  //***HOLONOMIC MOTOR CONTROLS (ORIENTED FACING THE ROBOT FORWARDS)
  //FWD
    //LFT | RGHT | LFT | RGHT
  //BCK
    //RGHT | LFT | RGHT | LFT
  //LFT
    //LFT | LFT | LFT | LFT
  //RGHT
    //RGHT | RGHT | RGHT | RGHT

 while(true) {
   if(RemoteControlCodeEnabled) {
     // calculate the drivetrain motor velocities from the controller joystick axies
    
    if (upDown > 5 || upDown < -5) {
      //have bot move in vertical direction

      DrivetrainNeedsToBeStopped_Controller1 = true;
    }
    if (leftRight > 5 || leftRight < -5) {
      //have bot move in horizontal direction

      DrivetrainNeedsToBeStopped_Controller1 = true;
    }
    if (botRot > 5 || botRot < -5) {
      //have bot rotate in direction

      DrivetrainNeedsToBeStopped_Controller1 = true;
    }
    
    if (!upDown && !leftRight && !botRot){
      if (DrivetrainNeedsToBeStopped_Controller1 == true){
        //stop motor I
        //stop motor II
        //stop motor III
        //stop motor IV
      
        DrivetrainNeedsToBeStopped_Controller1 = false;
      }
    }
   }
 }
}
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);

}