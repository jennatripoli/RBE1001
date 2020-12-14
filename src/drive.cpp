#include <drive.h>

/**
 * Turn in place.
 * @param angle  how far to turn, in degrees (positive = clockwise)
 **/
void Drive::hardTurn(double angle) {
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);

  double degreesToMove = (angle * track) / diam;
  leftMotor.startMoveFor(degreesToMove, 100);
  rightMotor.moveFor(-degreesToMove, 100);

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
}

/**
 * Turn corners by only powering one drive motor.
 * @param angle  how far to turn, in degrees (positive = clockwise)
 **/
void Drive::softTurn(double angle) {  
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  
  double degreesToMove = (2 * angle * track) / diam;
  if (angle >= 0) leftMotor.moveFor(degreesToMove, 150);  // turn right
  else rightMotor.moveFor(-degreesToMove, 150);  // turn left

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
}

/**
 * Drive in a straight line.
 * @param distance  how far to drive, in inches (positive = forward)
 **/
void Drive::straight(double distance) {
  double degreesToMove = (360 * distance) / (diam * PI);
  leftMotor.startMoveFor(degreesToMove, defaultSpeed);
  rightMotor.moveFor(degreesToMove, defaultSpeed);
}

/**
 * Stop both drive motors.
 **/
void Drive::stop(void) {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

/**
 * Follow the line on the field.
 * @param reflectance1  line sensor value of pin 1
 * @param reflectance2  line sensor value of pin 2
 **/
void Drive::lineFollow(int reflectance1, int reflectance2) {
  double error = reflectance1 - reflectance2;
  double effort = kP * error;
  rightMotor.setSpeed(defaultSpeed + effort);
  leftMotor.setSpeed(defaultSpeed - effort);
}