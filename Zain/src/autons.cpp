#include "vex.h"


/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0.001, 1.5, 0);
  chassis.set_heading_constants(6, 0.4, 0, 1, 0);
  chassis.set_turn_constants(9, 0.25, 0.01, 1.1, 15);
  chassis.set_swing_constants(12, .3, 0.001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 200, 5000);
  chassis.set_turn_exit_conditions(1, 200, 1500);
  chassis.set_swing_exit_conditions(1, 200, 3000);
}


void redAWP() {
  // Initialising Motors
  arm.setVelocity(100, rpm);
  lower.setVelocity(200, rpm);
  upper.setVelocity(600, rpm);
  chassis.set_heading(0);

  // Alliance Stake
  arm.spin(reverse);
  wait(1, seconds);
  arm.spinFor(forward, 90, degrees);

  // Mogo
  chassis.drive_distance(-55);
  chassis.turn_to_angle(85);
  chassis.drive_distance(-38);
  mogoClamp.set(true);

  // Ring 1
  chassis.turn_to_angle(140);
  lower.spin(forward);
  upper.spin(forward);
  chassis.drive_distance(50);
  chassis.set_heading(0);
  chassis.turn_to_angle(-40);
  goalRush.set(true);

  // Rings 2 and 3
  chassis.drive_distance(12);
  chassis.turn_to_angle(150);
  chassis.drive_distance(70);
  chassis.turn_to_angle(180);
  chassis.drive_max_voltage = 7.6;
  chassis.drive_distance(120);
}


void blueAWP() {
  // Initialising Motors
  arm.setVelocity(100, rpm);
  lower.setVelocity(200, rpm);
  upper.setVelocity(600, rpm);
  chassis.set_heading(0);

  // Alliance Stake
  arm.spin(reverse);
  wait(1, seconds);
  arm.spinFor(forward, 90, degrees);

  // Mogo
  chassis.drive_distance(-55);
  chassis.turn_to_angle(-85);
  chassis.drive_distance(-38);
  mogoClamp.set(true);

  // Ring 1
  chassis.turn_to_angle(-140);
  lower.spin(forward);
  upper.spin(forward);
  chassis.drive_distance(50);
  chassis.set_heading(0);
  chassis.turn_to_angle(-60);
  goalRush.set(true);

  // Rings 2 and 3
  chassis.drive_distance(30);
  chassis.turn_to_angle(-210);
  chassis.turn_to_angle(-180);
  chassis.drive_max_voltage = 7.6;
  chassis.drive_distance(120);
}