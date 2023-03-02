#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
inertial inert = inertial(PORT1);
motor motor_a = motor(PORT12, ratio18_1, false);
motor motor_b = motor(PORT13, ratio18_1, false);
motor motor_c = motor(PORT14, ratio18_1, false);
motor motor_d = motor(PORT15, ratio18_1, false);
motor flywheel_1 = motor(PORT19, ratio18_1, false);
motor flywheel_2 = motor(PORT20, ratio18_1, false);
motor intake_1 = motor(PORT17, ratio18_1, false);
motor intake_2 = motor(PORT18, ratio18_1, true);
pneumatics indexer = pneumatics(Brain.ThreeWirePort.A);
gps GPS = gps(PORT16, -80.00, 90.00, mm, 90);
optical opt = optical (PORT10);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;

double flywheel_mult;
float flywheel_voltage = 10; // 10 VOLTS AT THE MOST!
double overshoot_target = 11; // 11 VOLTS AT THE MOST!
double undershoot_target = 9; // 9 VOLTS AT THE MOST!
bool spin_flywheel;

void flywheel_05(){
  if (flywheel_mult <= 1.0) {
    flywheel_mult += 0.005;
  }
}

void flywheel_01(){
  if (flywheel_mult <= 1.0) {
    flywheel_mult += 0.001;
  }
}

void flywheel_05_n(){
  if (flywheel_mult >= 0.0) {
    flywheel_mult -= 0.005;
  }
}

void flywheel_01_n(){
  if (flywheel_mult >= 0.0) {
    flywheel_mult -= 0.001;
  }
}

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {

      Brain.Screen.clearLine();
    
    //FLYWHEEL (COMMENTED OUT & MOVED TO MAIN.CPP)

    
    Controller1.ButtonUp.pressed(flywheel_05);
    Controller1.ButtonDown.pressed(flywheel_05_n);
    Controller1.ButtonLeft.pressed(flywheel_01);
    Controller1.ButtonRight.pressed(flywheel_01_n);
    
     
    if (Controller1.ButtonB.pressing()) 
    {
      flywheel_mult = 0.5;      
      Controller1LeftShoulderControlMotorsStopped = false;
    }  else if (Controller1.ButtonA.pressing()) 
    {
      flywheel_mult = 1.0;
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1LeftShoulderControlMotorsStopped = false;
    } else if (Controller1.ButtonY.pressing()) 
    {
      flywheel_mult = 0.85;
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1LeftShoulderControlMotorsStopped = false;
    } else if (Controller1.ButtonX.pressing()) 
    {
      flywheel_mult = 0;
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1LeftShoulderControlMotorsStopped = false;
    } 

    /* 
    flywheel_1.spin(forward, flywheel_voltage * flywheel_mult, volt);
    flywheel_2.spin(reverse, flywheel_voltage * flywheel_mult, volt);
    */
 
    // INDEXER AND EXPANSION (COMMENTED OUT & MOVED TO MAIN.CPP)
    
    // Indexer control

    /*Brain.Screen.print(flywheel_1.velocity(percent));
    Brain.Screen.newLine();
    Brain.Screen.print(flywheel_1.voltage());
    Brain.Screen.newLine();
    Brain.Screen.print("");
    // wait before repeating the process*/

    //Move();


    wait(20, msec);
    }
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}