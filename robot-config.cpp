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
motor flywheel_1 = motor(PORT7, ratio18_1, false); 
motor flywheel_2 = motor(PORT8, ratio18_1, false); 
motor intake = motor(PORT11, ratio18_1, false); 
//sensors
rotation odometry_encoder = rotation(PORT5, false);
distance odometry_distance = distance(PORT20);
inertial inertia_sensor = inertial(PORT6);
//pneumatics
digital_out indexer = digital_out(Brain.ThreeWirePort.A);
// flywheel stuff
bool flywheel_powered = false;
int flywheel_velocity = 0;
// pi
float pi = 3.1415926535;

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
bool DrivetrainNeedsToBeStopped_Controller1 = true;

double radians_to_degrees(float radians){
  double degrees = radians / pi * 180;
  return degrees;
}

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

void MoveAuton (float velocity_auton, float angle_auton, float rotation_auton, float distance_auton){ 
  // angle must be given in radians, velocity is given in percentage

  // the distance that each axis has to push the robot
  double y_distance = distance_auton * sin(angle_auton);
  double x_distance = distance_auton * cos(angle_auton);

  // how many times the wheels have to rotate to move y_distance or x_distance
  double y_rotations = y_distance / 4*pi;
  double x_rotations = x_distance / 4*pi;

  // determine the magnatude of velocity applied by the front two wheels
  double motor_a_c_vel = velocity_auton * sin(angle_auton); // wheel axis 1
  double motor_b_d_vel = velocity_auton * cos(angle_auton); // wheel axis 2

  // sets the velocities of the wheels to the magnatudes of the velocities
  motor_a.setVelocity(motor_a_c_vel + rotation_auton, percent);
  motor_b.setVelocity(motor_b_d_vel + rotation_auton, percent);
  motor_c.setVelocity(-motor_a_c_vel - rotation_auton, percent);
  motor_d.setVelocity(-motor_b_d_vel - rotation_auton, percent);

  // spin the wheels in the angular velocities. 
  motor_a.spinTo((radians_to_degrees(2 * pi * y_rotations)), degrees);
  motor_b.spinTo((radians_to_degrees(2 * pi * x_rotations)), degrees);
  motor_c.spinTo(-(radians_to_degrees(2 * pi * y_rotations)), degrees);
  motor_d.spinTo(-(radians_to_degrees(2 * pi * x_rotations)), degrees);
}

int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values

  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      Move();
      // spin flywheel at the velocity set by input
      flywheel_1.spin(forward, flywheel_velocity, percent); 
      flywheel_2.spin(reverse, flywheel_velocity, percent); 
      
      // power flywheel with B
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
          flywheel_velocity = 65;
        }
      } 
      
      // use pneumatics with R1
      if (Controller1.ButtonR1.pressing()) {
        //bring flywheel to 85 percent
        flywheel_velocity = 85;
        //wait 20 msec
        wait(20, msec);
        // push indexer out
        indexer.set(true);
        // wait 20 milliseconds
        wait(20, msec);
        // pull indexer in
        indexer.set(false);
        // reset motor speed
        flywheel_velocity = 65;
      }
    }
  }
}

void vexcodeInit( void ) {
  // nothing to initialize
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}