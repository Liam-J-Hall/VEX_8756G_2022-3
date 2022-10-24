#include "vex.h"
#include "robot-config.cpp" // !! REMOVE IF NECESARY !!

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  flywheel_1.setBrake(coast);
  flywheel_2.setBrake(coast); 
}

void autonomous(void) {
  // power flywheel
  flywheel_velocity = 75;
  flywheel_1.spin(forward, flywheel_velocity, percent);
  flywheel_2.spin(reverse, flywheel_velocity, percent); 
  flywheel_powered = true;

  // move to firing position
  MoveAuton(100, pi/2, 0, 33); // max velocity, 90 degrees, no rotation, 33 inches

  // rotate towards goal
  MoveAuton(100, 0, pi/2, 12); // !! LOOK INTO THE RADIUS OF ROBOT !!

  // shoot 2 preloads
  for (int i=0; i<2; i++){
    // push indexer out
    indexer.set(true);
    // wait 20 milliseconds
    wait(20, msec);
    // pull indexer in
    indexer.set(false);
    //wait 20 milliseconds
    wait(20, msec);
  }

  //turn on intake
  intake.spin(forward, 65, percent);
  
  // move towards the stack of discs (3)
  // collect stack of discs (3)
  MoveAuton(0,0,0,0); // !! CHANGE NUMBERS !!

  // move to firing position
  MoveAuton(0,0,0,0); // !! CHANGE NUMBERS !!

  // fire all stored discs (3) consecutively towards the goal
  for (int i=0; i<3; i++){
    // push indexer out
    indexer.set(true);
    // wait 20 milliseconds
    wait(20, msec);
    // pull indexer in
    indexer.set(false);
    //wait 20 milliseconds
    wait(20, msec);
  }
}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}