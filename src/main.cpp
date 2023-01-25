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

//radius of the wheel on the drive base in inches
const double WHEEL_RADIUS = 3.5/2;

//radius of the robot from wheel to center in inches
const double ROBOT_RADIUS = 15/2;


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
  inert.calibrate();
  inert.setHeading(GPS.heading(), degrees);
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

void display_position()
{
    Brain.Screen.newLine();

    Brain.Screen.print("(");
    Brain.Screen.print(GPS.xPosition());
    Brain.Screen.print(", ");
    Brain.Screen.print(GPS.yPosition());
    Brain.Screen.print(", angle:");
    Brain.Screen.print(inert.heading());
    Brain.Screen.print(")");
}

double m_angle[4] = {dtr(GPS.heading()) - pi/4, dtr(GPS.heading()) + pi/4, dtr(GPS.heading()) + 3*pi/4, dtr(GPS.heading()) - 3*pi/4};

double GPS_offset = 15;

double displacement;

void move_fwd(double goal_revolutions, double speed, bool wait = false) 
{
  motor_a.resetPosition();
  display_position();

  motor_a.spinFor(fwd, goal_revolutions, turns, speed - 20, rpm, false);
  motor_b.spinFor(reverse, goal_revolutions, turns, speed, rpm, false);
  motor_c.spinFor(reverse, goal_revolutions, turns, speed, rpm, false);
  motor_d.spinFor(fwd, goal_revolutions, turns, speed, rpm, wait);

  display_position();
}

void move_rev(double goal_revolutions, double speed, bool wait = false) 
{
  display_position();

  motor_a.spinFor(reverse, goal_revolutions, turns, speed, rpm, false);
  motor_b.spinFor(fwd, goal_revolutions, turns, speed, rpm, false);
  motor_c.spinFor(fwd, goal_revolutions, turns, speed, rpm, false);
  motor_d.spinFor(reverse, goal_revolutions, turns, speed, rpm, wait);
  
  display_position();
}

void strafe(bool is_left, double goal_revolutions, double speed, bool wait = false) 
{
  display_position();
  if (!is_left)
  {
    motor_a.spinFor(fwd, goal_revolutions, turns, speed, rpm, false);
    motor_b.spinFor(fwd, goal_revolutions, turns, speed, rpm, false);
    motor_c.spinFor(reverse, goal_revolutions, turns, speed, rpm, false);
    motor_d.spinFor(reverse, goal_revolutions, turns, speed, rpm, wait);
  } else {
    motor_a.spinFor(reverse, goal_revolutions, turns, speed, rpm, false);
    motor_b.spinFor(reverse, goal_revolutions, turns, speed, rpm, false);
    motor_c.spinFor(fwd, goal_revolutions, turns, speed, rpm, false);
    motor_d.spinFor(fwd, goal_revolutions, turns, speed, rpm, wait);
  }

  display_position();
}

/*void move_auton(double x_pos, double y_pos, double speed)
{
  // calculate angle that the robot must move to get from x_0 to x (and y_0 to y)
  double delta_y;
  double delta_x;
  double travel_angle;
  double m_a_speed;
  double m_b_speed;
  double n_gps_heading; // initial heading
  double d_gps_heading; //change in heading
  double u_heading = GPS.heading(); // usable heading
  double p_const = 0.2;
  double distance_from_goal;
  
  while ((GPS.xPosition() < x_pos - GPS_offset || GPS.xPosition() > x_pos + GPS_offset) || (GPS.yPosition() > y_pos + GPS_offset || GPS.yPosition() < y_pos - GPS_offset))
  {
    delta_y = y_pos - GPS.yPosition();
    delta_x = x_pos - GPS.xPosition();
    travel_angle = atan2(delta_x, delta_y);

    if (y_pos < GPS.yPosition()) {
      travel_angle += pi;
    }

    distance_from_goal = sqrt(pow(delta_y, 2) + pow(delta_x, 2));

    m_a_speed = speed * cos(fabs(travel_angle - pi/4 - (dtr(u_heading))));
    m_b_speed = speed * cos(fabs(travel_angle + pi/4 - (dtr(u_heading))));

    motor_a.spin(fwd, m_a_speed, percent);
    motor_b.spin(fwd, m_b_speed, percent);
    motor_c.spin(reverse, m_a_speed, percent);
    motor_d.spin(reverse, m_b_speed, percent);

    d_gps_heading = GPS.heading() - n_gps_heading;

    if (d_gps_heading == 0 && (motor_a.velocity(rpm) > 0 || motor_b.velocity(rpm) > 0)) {
      u_heading = inert.heading();
    } else {
      u_heading = GPS.heading();
    }
    
    n_gps_heading = GPS.heading();

    wait(10, msec);
  }

  motor_a.stop();
  motor_b.stop();
  motor_c.stop();
  motor_d.stop();
}*/

bool is_within_bounds (double x_bound, double y_bound, double error) {
  if (x_bound + error < GPS.xPosition(mm) || y_bound + error < GPS.yPosition(mm) || x_bound - error > GPS.xPosition(mm) || y_bound - error > GPS.yPosition(mm))
  {
    return true;
  } 
  else 
  {
    return false;
  }
}

double d_x;
double d_y;

double travel_angle;

double m_a_speed;
double m_b_speed;
double m_c_speed;
double m_d_speed;
double bot_orientation;

void move_auton (double x_goal, double y_goal, double speed){
  

  while (is_within_bounds(x_goal, y_goal, 20))
  {
    d_x = x_goal - GPS.xPosition(mm);
    d_y = y_goal - GPS.yPosition(mm);
    
    travel_angle = atan2(d_x, d_y);
    bot_orientation = dtr(GPS.heading());

    if (bot_orientation < 0) {
      bot_orientation += 2*pi;
    }

    if (d_y < 0){
      travel_angle += pi;
    }
    
    m_a_speed = speed * cos(fabs((bot_orientation + pi/4) - travel_angle));
    m_b_speed = speed * cos(fabs((bot_orientation + 7*pi/4) - travel_angle));
    m_c_speed = speed * cos(fabs((bot_orientation + 5*pi/4) - travel_angle));
    m_d_speed = speed * cos(fabs((bot_orientation + 3*pi/4) - travel_angle));
    motor_a.spin(fwd, m_a_speed, percent);
    motor_b.spin(fwd, m_b_speed, percent);
    motor_c.spin(fwd, m_c_speed, percent);
    motor_d.spin(fwd, m_d_speed, percent);

    wait(100, msec);
  }

  motor_a.stop();
  motor_b.stop();
  motor_c.stop();
  motor_d.stop();
}


//turns the robot the provided degrees
//assumes zero slipping
//direction is forward -> turn to right; backward -> left
//angle in degrees
void turn_angle(directionType direction, double angle_to_turn, bool stop = true){

  //angle that the wheel needs to turn for a supplied angle
  //based on the calculation change in distance = change in angle * radius
  double wheel_angle  = angle_to_turn * (ROBOT_RADIUS/WHEEL_RADIUS);
  
  //turn all 4 motors to turn robot
    motor_a.spinFor(direction, wheel_angle, degrees, false);
    motor_b.spinFor(direction, wheel_angle, degrees, false);
    motor_c.spinFor(direction, wheel_angle, degrees, false);
    motor_d.spinFor(direction, wheel_angle, degrees, stop);
}

//change from current position
//delta_x and delta_y measured in inches
void move_auton_rel_delta_xy(double delta_x, double delta_y, bool stop = true){

    //degrees that the wheels must turn to reach y
    double wheel_angle_ac = (delta_y/WHEEL_RADIUS) * (180/pi);
    //degrees that the wheels must turn to reach x
    double wheel_angle_db = (delta_x/WHEEL_RADIUS)  * (180/pi);

    //move the robot to position
    //x component motors
    motor_a.spinFor(forward, wheel_angle_ac, degrees, false);
    motor_c.spinFor(reverse, wheel_angle_ac, degrees, false);
    //y component motors
    motor_b.spinFor(forward, wheel_angle_db, degrees, false);
    motor_d.spinFor(reverse, wheel_angle_db, degrees, stop);
}

//heading is angle from motor a
void move_auton_delta_xy(double heading, double delta_x, double delta_y, bool stop = true){

  //heading relative in order to find the 
  double rel_heading =  heading;
  double rel_heading_magnitude = sqrt(pow(delta_x,2) + pow(delta_y,2));
  //relative to robot change in x
  double rel_delta_x = rel_heading_magnitude*sin( dtr(rel_heading) );
  //relative to robot change in y
  double rel_delta_y = rel_heading_magnitude*cos( dtr(rel_heading) );

 //degrees that the wheels must turn to reach y
    double wheel_angle_ac = (rel_delta_y/WHEEL_RADIUS) * (180/pi);
    //degrees that the wheels must turn to reach x
    double wheel_angle_db = (rel_delta_x/WHEEL_RADIUS)  * (180/pi);

    //move the robot to position
    //x component motors
    motor_a.spinFor(forward, wheel_angle_ac, degrees, false);
    motor_c.spinFor(reverse, wheel_angle_ac, degrees, false);
    //y component motors
    motor_b.spinFor(forward, wheel_angle_db, degrees, false);
    motor_d.spinFor(reverse, wheel_angle_db, degrees, stop);
    }

//converts the GPS from [0,180] and [-180,0] to [0,360] to be more usable
double GPSfix(double heading){
  if(heading<0){
    return heading + 360;
  }
  else{
    return heading;
  }
}

//function for moving to absolute coordinates on the field
void move_auton_xy(double x, double y, bool stop = true){  
  move_auton_delta_xy(GPSfix(GPS.heading())+135, (x-GPS.xPosition())/25.4, (y-GPS.yPosition())/25.4, stop);
}

//stop all 4 drivebase motors
void drive_stop(){
  motor_a.stop();
  motor_b.stop();
  motor_c.stop();
  motor_d.stop();
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  move_auton_xy(0, 0);

  //true if right in front of roller, false if not. changes based on needs of auton
  bool roller = false;
  if(!roller){
    //move to in front of roller
  }
  //move to touch roller
  //spin roller

  //move from roller
  //turn towards goal
  //move into position
  //spin up flywheel
  //launch disk
  //spin up flywheel 2
  //launch disk 2

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

double c_filter (double axis_value) {
  return pow(axis_value, 3) / 10000;
}

extern void Move()
{
  int motor_a_vel = c_filter(Controller1.Axis3.position()) + c_filter(Controller1.Axis4.position()) + c_filter(Controller1.Axis1.position());
  int motor_b_vel = -c_filter(Controller1.Axis3.position()) + c_filter(Controller1.Axis4.position()) + c_filter(Controller1.Axis1.position());
  int motor_c_vel = -c_filter(Controller1.Axis3.position()) - c_filter(Controller1.Axis4.position()) + c_filter(Controller1.Axis1.position());
  int motor_d_vel = c_filter(Controller1.Axis3.position()) - c_filter(Controller1.Axis4.position()) + c_filter(Controller1.Axis1.position());

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

  if (Competition.isDriverControl() == true || Competition.isAutonomous() == true){
    Brain.Screen.clearLine();
    Brain.Screen.newLine();
    Brain.Screen.print("(");
    Brain.Screen.print(GPS.xPosition());
    Brain.Screen.print(", ");
    Brain.Screen.print(GPS.yPosition());
    Brain.Screen.print(", angle:");
    Brain.Screen.print(inert.heading());
    Brain.Screen.print(")");

    Move();

    if (Controller1.ButtonUp.pressing()) {
      motor_a.spin(fwd, 50, percent);
      motor_b.spin(reverse, 50, percent);
      motor_c.spin(reverse, 50, percent);
      motor_d.spin(fwd, 50, percent);
    } 
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
