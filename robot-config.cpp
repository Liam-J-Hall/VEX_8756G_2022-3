#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
// VEXcode device constructors
controller Controller1 = controller(primary);
//driving motors
motor motor_a = motor(PORT1, ratio18_1, false); 
motor motor_b = motor(PORT9, ratio18_1, false);
motor motor_c = motor(PORT10, ratio18_1, false);
motor motor_d = motor(PORT2, ratio18_1, false);
//other motors
motor flywheel_1 = motor(PORT7, ratio18_1, false); //18_1 is a placeholder, gear ratio could be more or less
motor flywheel_2 = motor(PORT8, ratio18_1, false); //18_1 is a placeholder, gear ratio could be more or less
motor intake = motor(PORT11, ratio18_1, false); //18_1 is a placeholder, gear ratio could be more or less
//sensors
rotation odometry_encoder = rotation(PORT5, false);
distance odometry_distance = distance(PORT6);
// pi
float pi = 3.141592;

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
bool DrivetrainNeedsToBeStopped_Controller1 = true;

/*
***CONTROLS***

AXIS 3 - MOVE VERTICALLY // motors 1-2
AXIS 4 - MOVE HORIZONTALLY // motors 3-4
AXIS 1 - ROTATE ROBOT // motors 1-4

R1 - SHOOT DISC // motor 5
B - POWER FLY WHEEL // motors 7-8

A - SPIN ROLLER // motor 6

X - EXPAND ROBOT // motor 9

UP ARROW - TURN ON/OFF INTAKE MOTOR // motor 10

__ - Set motor type to break //motors 1-4
*/

void Move(){
  int motor_a_vel = Controller1.Axis3.position() + Controller1.Axis4.position() + Controller1.Axis1.position();
  int motor_b_vel = -Controller1.Axis3.position() + Controller1.Axis4.position() + Controller1.Axis1.position();
  int motor_c_vel = -Controller1.Axis3.position() - Controller1.Axis4.position() + Controller1.Axis1.position();
  int motor_d_vel = Controller1.Axis3.position() - Controller1.Axis4.position() + Controller1.Axis1.position();

  motor_a.setVelocity(motor_a_vel, percent);
  motor_b.setVelocity(motor_b_vel, percent);
  motor_c.setVelocity(motor_c_vel, percent);
  motor_d.setVelocity(motor_d_vel, percent);

  motor_a.spin(forward);
  motor_b.spin(forward);
  motor_c.spin(forward);
  motor_d.spin(forward);
}

void MoveAuton (float velocity_auton, float angle_auton, float rotation_auton){ // angle must be given in radians, velocity is given in percentage

  //while ()

  // determine the magnatude of velocity applied by the front two wheels
  int motor_a_c_vel = velocity_auton * sin(angle_auton); // wheel axis 1
  int motor_b_d_vel = velocity_auton * cos(angle_auton); // wheel axis 2

  // sets the velocities of the wheels to the magnatudes of the velocities
  motor_a.setVelocity(motor_a_c_vel + rotation_auton, percent);
  motor_b.setVelocity(motor_b_d_vel + rotation_auton, percent);
  motor_c.setVelocity(motor_a_c_vel + rotation_auton, percent);
  motor_d.setVelocity(motor_b_d_vel + rotation_auton, percent);

  // spin the wheels in the angular velocities. 
  motor_a.spin(forward);
  motor_b.spin(forward);
  motor_c.spin(forward);
  motor_d.spin(forward);
}

int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values

  bool flywheel_powered = false;
  int flywheel_velocity = 0;

  while(true) {
    if(RemoteControlCodeEnabled) {
     // calculate the drivetrain motor velocities from the controller joystick axies
      Move();
      if (Controller1.ButtonB.pressing()) {
        if(flywheel_powered == true){
          // turns off flywheel
          flywheel_powered = false;
          // lower flywheel velocity
          flywheel_velocity = 0;
        } else {
          // turns on flywheel
          flywheel_powered = true;
          // raises the flywheel velocity
          flywheel_velocity = 100;
        }
      }
      flywheel_1.spin(forward, flywheel_velocity, percent); // !!! CHANGE DIRECTION LATER !!!
      flywheel_2.spin(reverse, flywheel_velocity, percent); // !!! CHANGE DIRECTION LATER !!!
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