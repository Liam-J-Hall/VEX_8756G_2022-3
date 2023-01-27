using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor motor_a;
extern controller Controller1;
extern motor motor_b;
extern motor motor_c;
extern motor motor_d;
extern motor flywheel_1;
extern motor flywheel_2;
extern motor intake_1;
extern motor intake_2;
extern rotation odometry_rotation;
extern distance odometry_distance;
extern pneumatics indexer;
extern pneumatics expansion;
extern gps GPS;
extern inertial inert;
extern rotation flywheel_r;
extern optical opt;
extern void Move();
extern double pi;
extern void launch_disc();
extern float flywheel_voltage;
extern double overshoot_target;
extern double undershoot_target;
extern bool spin_flywheel;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );