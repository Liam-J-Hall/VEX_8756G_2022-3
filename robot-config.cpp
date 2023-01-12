#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor motor_a = motor(PORT1, ratio18_1, false);
controller Controller1 = controller(primary);
motor motor_b = motor(PORT2, ratio18_1, false);
motor motor_c = motor(PORT3, ratio18_1, false);
motor motor_d = motor(PORT4, ratio18_1, false);
motor flywheel_1 = motor(PORT6, ratio18_1, false);
motor flywheel_2 = motor(PORT7, ratio18_1, false);
motor intake_1 = motor(PORT8, ratio18_1, false);
motor intake_2 = motor(PORT9, ratio18_1, true);
rotation odometry_rotation = rotation(PORT12, false);
distance odometry_distance = distance(PORT11);
pneumatics indexer = pneumatics(Brain.ThreeWirePort.A);
rotation flywheel_r = rotation (PORT13, false);
gps GPS = gps(PORT10, 130.00, 135.00, mm, 90);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;

float motor_speed = 85;
float flywheel_voltage = 10; // 10 VOLTS AT THE MOST!
double overshoot_target = 11; // 11 VOLTS AT THE MOST!
double undershoot_target = 9; // 9 VOLTS AT THE MOST!
bool spin_flywheel;

void do_spin_flywheel(){
  if (spin_flywheel == true){
    flywheel_1.spin(forward, flywheel_voltage, volt);
    flywheel_2.spin(reverse, flywheel_voltage, volt);
  }
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
        intake_1.spin(forward, motor_speed, percent);
        intake_2.spin(forward, motor_speed, percent);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        intake_1.spin(reverse, motor_speed, percent);
        intake_2.spin(reverse, motor_speed, percent);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        intake_1.stop();
        intake_2.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
      
    if (Controller1.ButtonB.pressing()) 
    {
      spin_flywheel = true;
      
      Controller1LeftShoulderControlMotorsStopped = false;
    }  else /*if (!Controller1LeftShoulderControlMotorsStopped) */
    {
      spin_flywheel = false;
      // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      Controller1LeftShoulderControlMotorsStopped = true;
    }

    if (Controller1.ButtonL2.pressing()) {
      indexer.open();
    } else {
      indexer.close();
    }

    /*Brain.Screen.print(flywheel_1.velocity(percent));
    Brain.Screen.newLine();
    Brain.Screen.print(flywheel_1.voltage());
    Brain.Screen.newLine();
    Brain.Screen.print("");
    // wait before repeating the process*/

    //Move();
    do_spin_flywheel();


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