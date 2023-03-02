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
double P_CONST = 0.05;

//radius of the wheel on the drive base in inches
const double WHEEL_RADIUS = 3.5/2;

//radius of the robot from wheel to center in inches
const double ROBOT_RADIUS = 15/2;

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

double heading_avg () 
{
  double sum_heading =  GPS.heading() + inert.heading();
  return sum_heading *  0.5;
}

int bang_bang_motor_controller(){
  
  if (flywheel_1.velocity(rpm) < undershoot_target) {
    flywheel_voltage += 0.2;
  } else if (flywheel_1.velocity(rpm) > overshoot_target) {
    flywheel_voltage -= 0.;
  }

  return 0;
}

int controller_screen_manager()
{
  Brain.Screen.clearScreen();
  while(1)
  {
    if (Brain.Battery.capacity(percent) <= 30)
    {
      Brain.Screen.clearScreen(color::orange);
      Controller1.Screen.clearScreen();
      wait(200, msec);
      Brain.Screen.printAt(90,135, ">> RECHARGE BATTERY <<");
      Controller1.rumble("--");
      Controller1.Screen.setCursor(1, 2);
      Controller1.Screen.print("RECHARGE");
      Controller1.Screen.setCursor(2, 8);
      Controller1.Screen.print("BATTERY");
    } else {

      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("BATT");
      Controller1.Screen.setCursor(1, 6);
      Controller1.Screen.print(Brain.Battery.capacity(percent));
       
      Controller1.Screen.setCursor(1, 9);
      Controller1.Screen.print("|");
      
      
      Controller1.Screen.setCursor(1, 11);
      Controller1.Screen.print("MOTC");
      Controller1.Screen.setCursor(1, 16);
      Controller1.Screen.print((motor_a.temperature(celsius) + motor_b.temperature(celsius) + motor_c.temperature(celsius) + motor_d.temperature(celsius)) / 4);
      
      
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("I-TRQ");
      Controller1.Screen.setCursor(2, 10);
      Controller1.Screen.print(intake_1.torque());
      

      
      Controller1.Screen.setCursor(3, 1);
      Controller1.Screen.print("FLWL");
      Controller1.Screen.setCursor(3, 6);
      Controller1.Screen.print(flywheel_1.velocity(percent));
      
    }
    wait(100, msec);
  }
}



void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Controller1.Screen.clearScreen();

  
  GPS.calibrate();
  inert.calibrate();
  task c_manager = task(controller_screen_manager);

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

double num_of_turns;

// goal_distance = inches
void move_fwd(double goal_distance, double speed, bool wait = false) 
{
  motor_a.resetPosition();
  display_position();
  
  num_of_turns = goal_distance * cos(pi/4) / 2 * pi * WHEEL_RADIUS;

  motor_a.spinFor(fwd, num_of_turns, turns, speed, rpm, false);
  motor_b.spinFor(reverse, num_of_turns, turns, speed, rpm, false);
  motor_c.spinFor(reverse, num_of_turns, turns, speed, rpm, false);
  motor_d.spinFor(fwd, num_of_turns, turns, speed, rpm, wait);

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



// PID proportion control for the turn_to() function (defined within the function, declared here)
double proportion_turn;
double dot_prod_of_angles;
double g_angle_correction;

// turns the robot to the specific angle it needs to be
// goal_angle is measured in degrees
// speed is measured in percent (percent of maximum motor voltage)
void turn_to (double goal_angle, double speed)
{
  // Correction
  g_angle_correction = goal_angle * 0.5;

  // Keeps the motors from being affected by outside forces
  motor_a.setBrake(hold);
  motor_b.setBrake(hold);
  motor_c.setBrake(hold);
  motor_d.setBrake(hold);

  // Spins one direction while the current angle is less than the goal angle
  while (heading_avg() < g_angle_correction - 3)
  {
    
    proportion_turn = P_CONST * fabs(heading_avg() - g_angle_correction); 
    
    motor_a.spin(fwd, speed * proportion_turn, percent);
    motor_b.spin(fwd, speed * proportion_turn, percent);
    motor_c.spin(fwd, speed * proportion_turn, percent);
    motor_d.spin(fwd, speed, percent);
    
    wait(20, msec);
  }

  // Spins the other direction while the current angle is greater than the goal angle
  while (heading_avg() > g_angle_correction + 3)
  {
    proportion_turn = P_CONST * fabs(heading_avg() - g_angle_correction);

    motor_a.spin(reverse, speed * proportion_turn, percent);
    motor_b.spin(reverse, speed * proportion_turn, percent);
    motor_c.spin(reverse, speed * proportion_turn, percent);
    motor_d.spin(reverse, speed * proportion_turn, percent);
    
    wait(20, msec);
  }

  // Stops the motors and resets the brake types
  motor_a.stop();
  motor_b.stop();
  motor_c.stop();
  motor_d.stop();

  motor_a.setBrake(coast);
  motor_b.setBrake(coast);
  motor_c.setBrake(coast);
  motor_d.setBrake(coast);
}

/*void move_auton(double x_pos, double y_pos, double speed){
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
    return false;
  } 
  else 
  {
    return true;
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

/*double determine_angle(double x_goal, double y_goal) {
  d_x = x_goal - GPS.xPosition(mm);
  d_y = y_goal - GPS.yPosition(mm);
  
  travel_angle = atan2(d_y, d_x);

  if (d_x < 0){
    travel_angle += pi;
  }

  return travel_angle;
}*/

double rtd (double radians) 
{
  return radians * 180 / pi;
}

void turn_angle(directionType direction, double angle_to_turn, double speed, bool wait = true)
{

  //angle that the wheel needs to turn for a supplied angle
  //based on the calculation change in distance = change in angle * radius
  double wheel_angle  = angle_to_turn * (ROBOT_RADIUS/WHEEL_RADIUS);
  
  //turn all 4 motors to turn robot
    motor_a.spinFor(direction, wheel_angle, degrees, speed, rpm, false);
    motor_b.spinFor(direction, wheel_angle, degrees, speed, rpm, false);
    motor_c.spinFor(direction, wheel_angle, degrees, speed, rpm, false);
    motor_d.spinFor(direction, wheel_angle, degrees, speed, rpm, wait);
}

double angle_to_move;
directionType turning_dir;
double bot_heading;
double goal_angle;

void move_auton (double x_goal, double y_goal, double speed) {
  while (is_within_bounds(x_goal, y_goal, 20)) {
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
    m_b_speed = speed * cos(fabs((bot_orientation + 3*pi/4) - travel_angle));
    
    motor_a.spin(fwd, m_a_speed, percent);
    motor_b.spin(reverse, m_b_speed, percent);
    motor_c.spin(reverse, -1 * m_a_speed, percent);
    motor_d.spin(fwd, -1 * m_b_speed, percent);

    wait(100, msec);
  }

  motor_a.stop();
  motor_b.stop();
  motor_c.stop();
  motor_d.stop();
}

// shoots a number of discs defined here
void shoot_disc (int discs_fired, double flywheel_volts)
{
  flywheel_1.spin(forward, flywheel_volts, volt);
  flywheel_2.spin(reverse, flywheel_volts, volt);
  
  wait(2000, msec);
  for(int i=0; i < discs_fired; i++)
  {
    indexer.open();
    wait(100, msec);
    indexer.close();
    wait(1000, msec);
  }
}

void flywheel_stop() {
  flywheel_1.stop();
  flywheel_2.stop();
}

void spin_roller_2 (double s_time) {
  motor_a.spin(fwd, 20, rpm);
  motor_b.spin(reverse, 20, rpm);
  motor_c.spin(reverse, 20, rpm);
  motor_d.spin(fwd, 20, rpm);

  intake_2.spin(fwd, 100, rpm);

  wait(s_time, msec);

  motor_a.stop();
  motor_b.stop();
  motor_c.stop();
  motor_d.stop();
  
  intake_2.stop();
}



bool is_color_red;
vex::color roller_color;

float RED1[2] = {0, 99};
float RED2[2] = {300, 360};
float BLUE[2] = {100, 299};
float goal_color[2];


// turns the roller to the opposite color from what it does
// ERROR: only turns from blue up to red up
float spin_roller () {
  opt.setLightPower(100, percent);

  // drive fwd until the torque on the drive motor indicates that it's pressing w the roller. 
  while (!opt.isNearObject())
  {
    motor_a.spin(fwd);
    motor_b.spin(reverse);
    motor_c.spin(reverse);
    motor_d.spin(fwd);
    intake_2.spinFor(100, degrees, 100, rpm, true);
  }

  if (opt.hue() <= RED1[1] || opt.hue() >= RED2[0]) {
    is_color_red = true;
  } else {
    is_color_red = false;
  }

  motor_a.setBrake(hold);
  motor_b.setBrake(hold);
  motor_c.setBrake(hold);
  motor_d.setBrake(hold);

  if (is_color_red == false) 
  {
    
    while (opt.hue() < RED2[0] || opt.hue() > RED1[1]) {
      intake_2.spin(forward, 100, rpm);

      motor_a.spin(fwd, 10, percent);
      motor_b.spin(reverse, 10, percent);
      motor_c.spin(reverse, 10, percent);
      motor_d.spin(fwd, 10, percent);

    }
  } else if (is_color_red == true) 
  {
    while (opt.hue() > RED2[0] || opt.hue() < RED1[1]) {
      intake_2.spin(forward, 100, rpm);

      motor_a.spin(fwd, 10, percent);
      motor_b.spin(reverse, 10, percent);
      motor_c.spin(reverse, 10, percent);
      motor_d.spin(fwd, 10, percent);
    } 
  }

  motor_a.stop();
  motor_b.stop();
  motor_c.stop();
  motor_d.stop();

  motor_a.setBrake(coast);
  motor_b.setBrake(coast);
  motor_c.setBrake(coast);
  motor_d.setBrake(coast);
  
  intake_2.stop();
  opt.setLightPower(0, percent);
  return 0;
}

///////////////////////////////
////// ROWAN'S CODE: //////////
///////////////////////////////

//determines actual angle instead of only 1 solution from arcsin
double determine_angle(double adjacent, double opposite){

  double hypo = sqrt(pow(adjacent, 2) + pow(opposite, 2));
  double actual_angle = asin(opposite/hypo);

  if(adjacent >= 0 && opposite >= 0){ //quadrant 1
      return actual_angle;
  } else if(adjacent <= 0 && opposite >=0){ //quadrant 2
      return pi - actual_angle; 
  } else if(adjacent <= 0 && opposite <=0){ //quadrant 3
    return pi-actual_angle;
  } else if(adjacent >=0 && opposite <= 0){ //quadrant 4
    return 2 * pi + actual_angle;
  } else {
    return 80085; //if things are wrong return a out of domain number for debugging
  }
}

//change from current position
//delta_x and delta_y measured in inches
void move_auton_rel_delta_xy(double delta_x, double delta_y, bool stop = true){

  //degrees that the wheels must turn to reach y
  double wheel_angle_ac = (delta_x/WHEEL_RADIUS) * (180/pi);
  //degrees that the wheels must turn to reach x
  double wheel_angle_db = (delta_y/WHEEL_RADIUS)  * (180/pi);

    //move the robot to position
    //x component motors
    motor_a.spinFor(forward, wheel_angle_ac, degrees, false);
    motor_c.spinFor(reverse, wheel_angle_ac, degrees, false);
    //y component motors
    motor_b.spinFor(reverse, wheel_angle_db, degrees, false);
    motor_d.spinFor(forward, wheel_angle_db, degrees, stop);
}

//heading is angle from motor a
void move_auton_delta_xy(double heading, double delta_x, double delta_y, bool stop = true){

  delta_x*=-1;
  //finds magnitude of the vector in the direction travelling
  double rel_heading_magnitude = sqrt(pow(delta_x,2) + pow(delta_y,2));

  //finds angle of the vector of travel and adjusts it so that the angle begins at the a motor
  //if this is going in the wrong direction then the pi/2 probably needs to be subtracted
  double rel_heading =  heading*pi/180+determine_angle(delta_x, delta_y);

  //relative to robot change in x
  //double rel_delta_x = rel_heading_magnitude*cos(rel_heading);
  double rel_delta_x = rel_heading_magnitude*cos(rel_heading);
  //relative to robot change in y
  double rel_delta_y = rel_heading_magnitude*sin(rel_heading);

 //degrees that the wheels must turn to reach x
  double wheel_angle_db = (rel_delta_x/WHEEL_RADIUS) * (180/pi);
  //degrees that the wheels must turn to reach y
  double wheel_angle_ac = (rel_delta_y/WHEEL_RADIUS)  * (180/pi);

  //move the robot to position
  //x component motors
  motor_a.spinFor(forward, wheel_angle_ac, degrees, false);
  motor_c.spinFor(reverse, wheel_angle_ac, degrees, false);

  //y component motors
  motor_b.spinFor(reverse, wheel_angle_db, degrees, false);
  motor_d.spinFor(forward, wheel_angle_db, degrees, stop);
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

bool spin_to_color_red;

void autonomous(void) {
  // .......................................................................... //
  //                    Insert autonomous user code here.                       //
  // .......................................................................... //

  
  // start flywheels
  flywheel_1.spin(fwd, 9.0, volt);
  flywheel_2.spin(reverse, 9.0, volt);

  // spin starting roller !
  spin_roller(); 
  
  // move back from the roller
  move_auton_delta_xy(-45, 0, -10);

  // turn 90 degrees
  turn_to(90, 25);

  // fire the two discs
  shoot_disc(2, 9.0);

  flywheel_1.stop();
  flywheel_2.stop();

  // move to the second, adjacent roller
  move_auton_delta_xy(-45, 18, 0);
  move_auton_delta_xy(-45, 0, 20);

  // spin the roller !
  spin_roller();

  // move to the other side of the field, to the opposite side roller
  move_auton_delta_xy(-45, 0, -30);
  move_auton_delta_xy(-45, 100, -100);

  // turn 180 degrees
  turn_to(180, 25);

  // spin the third roller !
  spin_roller();

  // move back from the roller
  move_auton_delta_xy(-45, 0, -30);

  // turn to fourth roller !
  turn_to(270, 25);

  // move to the second, adjacent roller
  move_auton_delta_xy(-45, 20, 18);

  // spin the fourth roller
  spin_roller();

  // back away from roller
  move_auton_delta_xy(-45, 0, -30);

  // turn around
  turn_to(360 -45, 25);

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

double hue_init;
double d_hue;
float intake_speed = 100;


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

    if (Controller1.ButtonL2.pressing()) {
      indexer.open();
    } else {
      indexer.close();
    } 

    d_hue = hue_init - opt.hue();
    // check the ButtonL1/ButtonL2 status to control intake_1
      if (Controller1.ButtonR1.pressing()) {
        intake_1.spin(forward, intake_speed, percent);
        intake_2.spin(forward, intake_speed, percent);
        if (fabs(d_hue) > 15 && opt.isNearObject() == true)
        {
          Controller1.rumble("..-.");
        }
      } else if (Controller1.ButtonR2.pressing()) {
        intake_1.spin(reverse, intake_speed, percent);
        intake_2.spin(reverse, intake_speed, percent);
        if (fabs(d_hue) > 15 && opt.isNearObject() == true)
        {
          Controller1.rumble("..-.");
        }
      } else {
        intake_1.stop();
        intake_2.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
      }

      if (Controller1.ButtonL1.pressing())
      {
        motor_a.setBrake(hold);
        motor_b.setBrake(hold);
        motor_c.setBrake(hold);
        motor_d.setBrake(hold);
      } else {
        motor_a.setBrake(coast);
        motor_b.setBrake(coast);
        motor_c.setBrake(coast);
        motor_d.setBrake(coast);
      }

      // if (Controller1.ButtonL1.pressing() && Controller1.ButtonL2.pressing() && Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()) {
        // Expansion here
      //}

    //Spin flywheel
    flywheel_1.spin(forward, flywheel_voltage * flywheel_mult, volt);
    flywheel_2.spin(forward, flywheel_voltage * flywheel_mult, volt);

    hue_init = opt.hue();

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
