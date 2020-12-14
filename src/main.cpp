#include <Arduino.h>
#include <RBE1001Lib.h>

// for rangefinder
Rangefinder rangefinder;
float distanceToBag = 5.0;  // how far away the robot should be from the bag when picking it up
int bagThreshold = 30;  // how far away the robot can be to detect a bag
int zoneThreshold = 3;

// button for starting program
const int buttonPin = BOOT_FLAG_PIN;
bool upDown = false;

// for driving
Motor leftMotor, rightMotor;
float diam = 2.75;  // diameter of drive wheels, in inches
float track = 5.875;
float defaultSpeed = 200;  // default speed for driving, in degrees/sec
float kP = 0.1;  // proportional constant for drive wheels (0.06 on Dilce's bot)

// for line following
const int reflectancePin1 = 39, reflectancePin2 = 36;
int reflectance1, reflectance2;  // output of the two line sensor pins
int threshold = 1000;  // minimum line sensor output for the line to be seen

//for servo arm
Servo armServo;
const int servoPin = 33;
int deliverA = 0, deliverB = 75, deliverC = 130;  // angles to deliver bags on zones A, B, and C

//state machine
enum ROBOT_STATES { LINE_FOLLOW_OUT, APPROACH_BAG, STREET_1, STREET_2, STREET_3, STREET_4, STREET_5, LINE_FOLLOW_CRUTCH, end };
int robotState;
int bagState = 0;  // 0 = Bag 1 (zone A), 1 = Bag 2 (zone B), 2 = Bag 3 (zone C)

// functions
void lineFollow(int reflectance1, int reflectance2);
void hardTurn(float angle);
void softTurn(float angle);
void straight(float distance);
double ultrasonicRead();  // TODO: where is this??
void pickUpBag(void);
void dropOffBag(void);
void updateRobotState(void);

void setup() {
  Motor::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  Serial.begin(115200);
  leftMotor.attach(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB);
  rightMotor.attach(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB);
  rangefinder.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
  armServo.attach(servoPin);
  pinMode(reflectancePin1, INPUT);
  pinMode(reflectancePin2, INPUT);
  robotState = LINE_FOLLOW_OUT;
}

/**
 * Follow the line on the field.
 * @param reflectance1  line sensor value of pin 1
 * @param reflectance2  line sensor value of pin 2
 **/
void lineFollow(int reflectance1 , int reflectance2) {
  float error = reflectance1 - reflectance2;
  float effort = kP * error;
  rightMotor.setSpeed(defaultSpeed + effort);
  leftMotor.setSpeed(defaultSpeed - effort);
}

/**
 * Turn in place.
 * @param angle  how far to turn, in degrees (positive = clockwise)
 **/
void hardTurn(float angle) {
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);

  float degreesToMove = (angle * track) / diam;
  leftMotor.startMoveFor(degreesToMove, 100);
  rightMotor.moveFor(-degreesToMove, 100);

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
}

/**
 * Turn corners by only powering one drive motor.
 * @param angle  how far to turn, in degrees (positive = clockwise)
 **/
void softTurn(double angle) {  
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  
  float degreesToMove = (2 * angle * track) / diam;
  if (angle >= 0) leftMotor.moveFor(degreesToMove, 150);  // turn right
  else rightMotor.moveFor(-degreesToMove, 150);  // turn left

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
}

/**
 * Drive in a straight line.
 * @param distance  how far to drive, in inches (positive = forward)
 **/
void straight(double distance) {
  float degreesToMove = (360 * distance) / (diam * PI);
  leftMotor.startMoveFor(spin, defaultSpeed);
  rightMotor.moveFor(spin, defaultSpeed);
}

/**
 * Center robot on bag using rangefinder, approach bag, and pick up bag.
 **/
void pickUpBag(void) {
  float leftEdge, rightEdge, error;
  armServo.write(0);
  delay(200);
  rangefinder.getDistanceCM();
  delay(100);

  // while object is out of range, turn clockwise in place
  while (rangefinder.getDistanceCM() > bagThreshold) {
    leftMotor.setSpeed(80);
    rightMotor.setSpeed(-80);
  }
  leftEdge = rightMotor.getCurrentDegrees(), rightEdge = leftEdge; // default for rightEdge causes no turning
  
  // wait for the object to get out of range
  while (rangefinder.getDistanceCM() < bagThreshold) {}
  rightEdge = rightMotor.getCurrentDegrees();  // degrees when object is out of range
  
  hardTurn((rightEdge - leftEdge) / 4);  // turn counterclockwise to center of bag

  // approach bag with proportional control
  while (rangefinder.getDistanceCM() > distanceToBag) {
    error = rangefinder.getDistanceCM();
    leftMotor.setEffort(error * kP / 4);  // divided by 2 on Dilce's bot
    rightMotor.setEffort(error * kP / 4);  // divided by 2 on Dilce's bot
  }

  armServo.write(0);
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  straight(-2);
  hardTurn(170);
  straight(-3.25);
  armServo.write(180);  // pick up bag
  delay(500);
}

/**
 * Deliver bag on its correct zone.
 **/
void dropOffBag(void) {
  hardTurn(180);
  bagState++;
  if (bagState == 1) armServo.write(deliverA);  // first bag delivered to zone A
  else if (bagState == 2) armServo.write(deliverB);  // second bag delivered to zone B
  else if (bagState == 3) {
    armServo.write(180);
    armServo.write(deliverC);  // third bag delivered to zone C
  }
  delay(1000);
}

/**
 * Switch statement to decide what the robot should do. 
 **/
void updateRobotState(void) {
  switch (robotState) {
  case LINE_FOLLOW_OUT:  // going down STREET_2 heading towards pick-up zone
    if ((reflectance1 >= threshold) && (reflectance2 >= threshold)) { // line sensor sees pick-up zone
      delay(50);
      softTurn(-85);
      robotState = APPROACH_BAG;
    } else lineFollow(reflectance1, reflectance2);
    break;

  case LINE_FOLLOW_CRUTCH:  // after bag drop-off, deciding how to turn to get back onto STREET_2
    if ((reflectance1 >= threshold) && (reflectance2 >= threshold)) {  // line sensor sees delivery zone intersection
      if (bagState == 1) {
        softTurn(-85);
        robotState = LINE_FOLLOW_OUT;
      } else if (bagState == 2) {
        straight(2);
        robotState = STREET_3;
      } else if (bagState == 3) {
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
        robotState = end;
      }
    } else lineFollow(reflectance1, reflectance2);
    break;
  
  case APPROACH_BAG:  // approaching to pick up a bag
    if ((reflectance1 > threshold) && (reflectance2 > threshold)) {  // line sensor sees T at pick-up zone
      straight(-5);
      hardTurn(-30);
      armServo.write(0);
      pickUpBag();
      straight(3);
      robotState = STREET_1;
    } else lineFollow(reflectance1, reflectance2);
    break;

  case STREET_1:  // leaving pick-up zone
    if ((reflectance1 > threshold) && (reflectance2 > threshold)) {  // line sensor seees end of pick-up street
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);
      delay(100);
      softTurn(85);
      robotState = STREET_2;
    } else lineFollow(reflectance1, reflectance2);
    break;

  case STREET_2:  // returning from STREET_1, deciding which street to deliver bag to
    if ((reflectance1 > threshold) && (reflectance2 > threshold)) {  // line sensor sees delivery zone intersection
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);
      delay(100);
      if (bagState == 0){
        softTurn(85);
        robotState = STREET_3;
      } else if (bagState == 1){
        softTurn(-85);
        robotState = STREET_4;
      } else if (bagState == 2){
        straight(2);
        robotState = STREET_5;
      }   
    } else lineFollow(reflectance1, reflectance2);
    break;

  case STREET_3:  // first bag drop-off on zone A and pick up of free-range bag
    if ((reflectance1 > threshold) && (reflectance2 > threshold)) {  // line sensor sees T at zone A
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);
      delay(100);
      if (bagState == 2) {  // for free-range bag
        hardTurn(90);
        straight(5);
        hardTurn(-30);
        armServo.write(0);
        delay(200);
        pickUpBag();  // pick up free-range bag
        hardTurn(-35);
        robotState = STREET_2;
      } else {
        dropOffBag();
        robotState = LINE_FOLLOW_CRUTCH;
      }
    } else lineFollow(reflectance1, reflectance2);
    break;

    case STREET_4:  // Second Bag drop off
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){
         leftMotor.setSpeed(0);
         rightMotor.setSpeed(0);
         delay(100);
         dropOffBag();
         robotState = LINE_FOLLOW_CRUTCH;
       }else{
         lineFollow(reflectance1, reflectance2);
       }
      break;

    case STREET_5:  // Third Bag drop off
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){
         leftMotor.setSpeed(0);
         rightMotor.setSpeed(0);
         delay(100);
         dropOffBag();
         robotState = LINE_FOLLOW_CRUTCH;
       }else{
         lineFollow(reflectance1, reflectance2);
       }

      break;

    case end: // Ends function and concludes program

      break;
  }
}

void loop() { 
 while(digitalRead(buttonPin)) {} //wait for button press
  delay (500); // Lets Robot Prep itself

  while(true){
   reflectance1=analogRead(reflectancePin1);
   reflectance2=analogRead(reflectancePin2);
   updateRobotState();
  }
}