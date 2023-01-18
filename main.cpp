/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// motor_a              motor         1               
// Controller1          controller                    
// motor_b              motor         2               
// motor_c              motor         3               
// motor_d              motor         4               
// flywheel_1           motor         6               
// flywheel_2           motor         7               
// intake_1             motor         8               
// intake_2             motor         9               
// odometry_rotation    rotation      10              
// odometry_distance    distance      11              
// inertial             inertial      5               
// pneumatics           digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
double pi = 3.1415926535;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

double radians_to_degrees(double radians){
  return radians * 180 / pi;
}

double dtr(double degrees){
  return degrees * pi / 180;
}

int bang_bang_motor_controller(){
  
  if (flywheel_1.velocity(rpm) < undershoot_target) {
    flywheel_voltage += 0.2;
  } else if (flywheel_1.velocity(rpm) > overshoot_target) {
    flywheel_voltage -= 0.;
  }

  return 0;
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  Brain.Screen.clearScreen();
  GPS.calibrate();
  task bang_bang = task(bang_bang_motor_controller);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


double y_distance;
double x_distance;
double y_rotations;
double x_rotations;
double r_rotations;
double motor_a_c_vel;
double motor_d_b_vel;

/*void MoveAuton (double velocity_auton, double angle_auton, double distance_auton)
{ 
  // angle must be given in radians, velocity is given in percentage

  // the distance that each axis has to push the robot
  y_distance = distance_auton * sin(angle_auton);
  x_distance = distance_auton * cos(angle_auton);
  Brain.Screen.print(y_distance);
  Brain.Screen.print("|");

  // how many times the wheels have to rotate to move y_distance or x_distance
  y_rotations = y_distance / 4*pi;
  x_rotations = x_distance / 4*pi;
  Brain.Screen.print(y_rotations);
  Brain.Screen.print("|");

  // determine the magnatude of velocity applied by the front two wheels
  motor_a_c_vel = velocity_auton * sin(angle_auton); // wheel axis 1
  motor_d_b_vel = velocity_auton * cos(angle_auton); // wheel axis 2
  Brain.Screen.print(motor_a_c_vel);
  Brain.Screen.print("|");

  // sets the velocities of the wheels to the magnatudes of the velocities
  motor_a.setVelocity(motor_a_c_vel, percent);
  motor_b.setVelocity(motor_d_b_vel, percent);
  motor_c.setVelocity(motor_a_c_vel, percent);
  motor_d.setVelocity(motor_d_b_vel, percent);
  Brain.Screen.print("MotorVel Set|");

  // spin the wheels in the angular velocities. 
  motor_a.rotateFor(forward, y_rotations * 360, degrees, false); // 2*pi multiplied by the number of 360 degree rotations
  motor_b.rotateFor(reverse, x_rotations * 360, degrees, false);
  motor_c.rotateFor(reverse, y_rotations * 360, degrees, false);
  motor_d.rotateFor(forward, x_rotations * 360, degrees, true);
  Brain.Screen.print("Moved");
}*/

/*
ADHD Fidget Junk Typing

as;lkdjfas;lkdjfsa;ldkfjasldfs;ladfkjas;dlfkjasldfkjasldkjfasldkjfa;sldkjfas;lkdjfas;lkdjfas;ldkfjas;ldkjf;alsdkfja;lsdkfjas;ldjf
sald;kfjas;ldkfjas;ldkfjas;lkdjfa;sldkjfa;slkdfja;sldkfjas;ldkfjas;ldkfjas;ldkfja;lkjfas;lkdjf;asldkfja;sldkfja;sldkfj
a;lskdjf;alskdjf;alskdjf;alskdjf;alskdjf;alskdjf
as;ldkfja;sldkfjasd;flaksjd;lfkasldkfja;sldkfja;sldkfja;sldkfja;sldkfja;sldkfja;sldkfja;sldkfja;sldkfja;sldkfja;lskdjf;alskdjf
a;lsdkjfa;sldfjka;lskdjf;alskdjf;alskdjf;alskdjf;alskdjf;alskdjf;alskdjf;alskdjf;alskdjf;lskdjf;alskdjf
*/

double determine_angle_sign(double a_const_input){
  double a_const_output;

  if (GPS.heading() > 180 && GPS.heading() < 361){
    a_const_output = -1 * a_const_input;
  } else {
    a_const_output = 1 * a_const_input;
  }

  return a_const_output;
}

void move_auton(double x_pos, double y_pos, double speed)
{
  // calculate angle that the robot must move to get from x_0 to x (and y_0 to y)
  double delta_y;
  double delta_x;
  double travel_angle;
  double axis_1_speed;
  double axis_2_speed;
  
  while ((GPS.xPosition() <! x_pos - 15 && GPS.xPosition() >! x_pos + 15) || !(GPS.yPosition() <! y_pos + 15 && GPS.yPosition() >! y_pos -15))
  { 
    delta_y = y_pos - GPS.yPosition();
    delta_x = x_pos - GPS.xPosition();
    travel_angle = atan(delta_y / delta_x);
    axis_1_speed = speed * cos(fabs(travel_angle - dtr(GPS.heading())) - pi/4);
    axis_2_speed = speed * cos(fabs(travel_angle - dtr(GPS.heading())) + pi/4);

    motor_a.spin(fwd, axis_1_speed, percent);
    motor_b.spin(reverse, axis_2_speed, percent);
    motor_c.spin(reverse, axis_1_speed, percent);
    motor_d.spin(fwd, axis_2_speed, percent);

    wait(10, msec);
  }

  motor_a.stop();
  motor_b.stop();
  motor_c.stop();
  motor_d.stop();
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  
  //motor_a.spin(fwd);

  move_auton(0, 0, 85);

  // Power Flywheel
  //flywheel_1.spin(forward, 65, percent);
  //flywheel_2.spin(reverse, 65, percent);

  // Move to roller
  //move_auton(1800, 1800, 65);

  // Use Roller
  //intake_2.spinFor(forward, 30, degrees);
  
  // Shoot disc

  //indexer.open();
  //wait(20, msec);
  //indexer.close();


   // power flywheel
  /*flywheel_velocity = 75;
  flywheel_1.spin(forward, flywheel_velocity, percent);
  flywheel_2.spin(reverse, flywheel_velocity, percent); 
  flywheel_powered = true;

  // move to firing position
  MoveAuton(100, pi/4, 33); // max velocity, 90 degrees, 33 inches

  // rotate towards goal
  TurnAuton(pi/4);

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
  
  // move towards the stack of discs (3); collect discs (3);
  TurnAuton(pi);
  MoveAuton(100, pi/4, 12); 
  //wait for discs to be collected
  wait(100 ,msec);

  // move to firing position
  MoveAuton(100, pi/4, 12); 
  TurnAuton(pi);

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

  // turn in orientation to use the far roller
  TurnAuton(-(3*pi/4));

  // drive to roller
  MoveAuton(100, atan(1.75/5), sqrt(5*5 + 1.75*1.75));

  // use roller
  intake.spinTo(radians_to_degrees(2*pi), degrees);*/

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
extern void Move()
{
  int motor_a_vel = Controller1.Axis3.position() + Controller1.Axis4.position() + Controller1.Axis1.position();
  int motor_b_vel = -Controller1.Axis3.position() + Controller1.Axis4.position() + Controller1.Axis1.position();
  int motor_c_vel = -Controller1.Axis3.position() - Controller1.Axis4.position() + Controller1.Axis1.position();
  int motor_d_vel = Controller1.Axis3.position() - Controller1.Axis4.position() + Controller1.Axis1.position();

  motor_a.spin(forward, motor_a_vel, percent);
  motor_b.spin(forward, motor_b_vel, percent);
  motor_c.spin(forward, motor_c_vel, percent);
  motor_d.spin(forward, motor_d_vel, percent);
}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

  if (Competition.isDriverControl() == true ){
    Move();
  }

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

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
