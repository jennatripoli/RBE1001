#include <Arduino.h>
#include <RBE1001Lib.h>
#inlcude < lineFollow.h>

Motor left, right;
Rangefinder rangefinder;
//Servo armServo;
//ESP32AnalogRead leftLine, rightLine;

const float kP = 0.02;
const float wheelDiameter = 7;                         // in cm
const float wheelCircumference = 3.14 * wheelDiameter; // in cm
const float degreesPerCm = 360.0 / wheelCircumference;

void setup()
{
    Serial.begin(9600);
    Motor::allocateTimer(0);
    //ESP32PWM::allocateTimer(1);
    left.attach(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB);
    right.attach(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB);
    rangefinder.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
    //armServo.attach(33);
    //leftLine.attach(36);
    //rightLine.attach(39);
}

/**
 * Drive straight at 180 deg/sec.
 * @param cm how far to drive, in cm (positive = forward)
 **/
void forward(float cm)
{
    float deg = cm * degreesPerCm;
    left.startMoveFor(deg, 180);
    right.moveFor(deg, 180);
}

/**
 * Turn in place a certain number of degrees.
 * @param degrees target change in angle, in degrees (positive = clockwise)
 **/
void turn(float degrees)
{
    float paramDegrees = ((degrees) / 360) * (wheelCircumference)*degreesPerCm * 2;
    left.startMoveFor(paramDegrees, 90);
    right.moveFor(-1 * paramDegrees, 90);
}

/**
 * Drive with proportional control to target distance and stop.
 * @param distanceFromObject target distance away from object, in cm
 **/

void driveToObject(float distanceFromObject)
{
    while (rangefinder.getDistanceCM() >= 5.08)
    {
        float error = rangefinder.getDistanceCM() - distanceFromObject;
        left.setEffort(error * kP * 1.15); // Numerical Constant to compensate for problematic motor
        right.setEffort(error * kP);
    }
    left.setEffort(0);
    right.setEffort(0);
}

/**
 * Find both edges of an object and turn to the center of the bag.
 * @param distanceFromObject max distance away from object, in cm
 **/
void turnToObject(float distanceFromObject)
{
    while (rangefinder.getDistanceCM() > distanceFromObject)
    { // while object is out of range
        left.setEffort(0.2);
        right.setEffort(-0.2);
    }
    float leftEdge = right.getCurrentDegrees(), rightEdge = leftEdge; // default for rightEdge causes no turning
    while (rangefinder.getDistanceCM() < distanceFromObject)
    {
    } // wait for the object to get out of range
    if (rangefinder.getDistanceCM() > distanceFromObject)
        rightEdge = right.getCurrentDegrees(); // when object is out of range
    Serial.print(leftEdge);
    Serial.print(rightEdge);
    turn((rightEdge - leftEdge) / 4); // turn ccw to center of object (average between the two edges)
}

void loop()
{

    delay(1000);
    turnToObject(30);
    driveToObject(5.08);
}
