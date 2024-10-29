// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// L1                   motor         3               
// L2                   motor         4               
// L3                   motor         5               
// R1                   motor         17              
// R2                   motor         7               
// R3                   motor         8               
// upper                motor         18              
// lower                motor         10              
// IMU                  inertial      15              
// mogoClamp            digital_out   A               
// goalRush             digital_out   B               
// arm                  motor         9               
// hang1                digital_out   F               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/
Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
ZERO_TRACKER_NO_ODOM,


//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(L1, L2, L3),

//Right Motors:
motor_group(R1, R2, R3),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT15,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
2.75,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
1.3333,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
350,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
1,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
6.25,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT16,

//Sideways tracker diameter (reverse to make the direction switch):
2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
-6.5

);

int current_auton_selection = 0;
bool auto_started = false;
bool mogoExtended = false;
bool rushExtended = false;
bool hangExtended = false;

/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */
void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();
  IMU.calibrate();

  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "JAR Template v1.2.0");
    Brain.Screen.printAt(5, 40, "Battery Percentage:");
    Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Selected Auton:");
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(5, 140, "RED AWP");
        break;
      case 1:
        Brain.Screen.printAt(5, 140, "BLUE AWP");
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 8){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void autonomous(void) {
  auto_started = true;
  switch(current_auton_selection){ 
    case 0:
      redAWP();
      break;
    case 1:
      blueAWP();
      break;
 }
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
void lowerIntakeForward() {
  lower.spin(forward);
  arm.setVelocity(200, rpm);
}
void lowerIntakeReverse() {
  lower.spin(reverse);
}
void lowerStop() {
  lower.stop();
}

void upperIntakeForward() {
  upper.spin(forward);
}
void upperIntakeReverse() {
  upper.spin(reverse);
}
void upperStop() {
  upper.stop();
}

void mogo() {
  if (mogoExtended == false) {
    mogoClamp.set(true);
    mogoExtended = true;
  }
  else {
    mogoClamp.set(false);
    mogoExtended = false;
  }
}

void rush() {
  if (rushExtended == false) {
    goalRush.set(true);
    rushExtended = true;
  }
  else {
    goalRush.set(false);
    rushExtended = false;
  }
}

void armForward() {
  arm.spin(forward);
}
void armReverse() {
  arm.spin(reverse);
}
void armStop() {
  arm.stop();
}

void hanging() {
  if (hangExtended == false) {
  hang1.set(true);
  hangExtended = true;
  }
  else {
    hang1.set(false);
    hangExtended = false;
  }
}

void wallMacro() {
  arm.spin(reverse);
  wait(1, seconds);
  arm.spinFor(forward, 90, degrees);
}


void usercontrol(void) {
  lowerStop();
  upperStop();
  L1.setVelocity(600, rpm);  
  L2.setVelocity(600, rpm);
  L3.setVelocity(600, rpm);
  R1.setVelocity(600, rpm);
  R2.setVelocity(600, rpm);
  R3.setVelocity(600, rpm);
  lower.setVelocity(600, rpm);
  upper.setVelocity(600, rpm);
  arm.setVelocity(100, rpm);
  arm.setBrake(hold);
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    //Replace this line with chassis.control_tank(); for tank drive 
    //or chassis.control_holonomic(); for holo drive.
    chassis.control_arcade();

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

  Controller1.ButtonL1.pressed(lowerIntakeForward);
  Controller1.ButtonR1.pressed(upperIntakeForward);
  Controller1.ButtonL2.pressed(lowerIntakeReverse);
  Controller1.ButtonR2.pressed(upperIntakeReverse);
  Controller1.ButtonUp.pressed(lowerStop);
  Controller1.ButtonDown.pressed(upperStop);

  Controller1.ButtonA.pressed(mogo);
  Controller1.ButtonB.pressed(rush);

  Controller1.ButtonLeft.pressed(armForward);
  Controller1.ButtonRight.pressed(armReverse);
  Controller1.ButtonLeft.released(armStop);
  Controller1.ButtonRight.released(armStop);

  Controller1.ButtonX.pressed(hanging);
  Controller1.ButtonY.pressed(wallMacro);


  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}