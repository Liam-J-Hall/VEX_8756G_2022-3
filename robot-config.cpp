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
pneumatics expansion = pneumatics(Brain.ThreeWirePort.C);
gps GPS = gps(PORT16, -80.00, 90.00, mm, 90);
optical opt = optical (PORT10);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;

float intake_speed = 85;
double flywheel_mult;
float flywheel_voltage = 10; // 10 VOLTS AT THE MOST!
double overshoot_target = 11; // 11 VOLTS AT THE MOST!
double undershoot_target = 9; // 9 VOLTS AT THE MOST!
bool spin_flywheel;

void do_spin_flywheel(){
  flywheel_1.spin(forward, flywheel_voltage * flywheel_mult, volt);
  flywheel_2.spin(reverse, flywheel_voltage * flywheel_mult, volt);
}

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {

      Brain.Screen.clearLine();

      // check the ButtonL1/ButtonL2 status to control intake_1
      if (Controller1.ButtonR1.pressing()) {
        intake_1.spin(forward, intake_speed, percent);
        intake_2.spin(forward, intake_speed, percent);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        intake_1.spin(reverse, intake_speed, percent);
        intake_2.spin(reverse, intake_speed, percent);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        intake_1.stop();
        intake_2.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
      
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
      flywheel_mult = 0.75;
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1LeftShoulderControlMotorsStopped = false;
    } else if (Controller1.ButtonX.pressing()) 
    {
      flywheel_mult = 0;
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1LeftShoulderControlMotorsStopped = false;
    } 

    do_spin_flywheel();


    if (Controller1.ButtonL2.pressing()) {
      indexer.open();
    } else {
      indexer.close();
    }

    if (Controller1.ButtonLeft.pressing()) {
      expansion.close();
    } else {
      expansion.open();
    }

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