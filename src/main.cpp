#include <Arduino.h>
#include <RBE1001Lib.h>
#include <drive.h>
#include <arm.h>

// objects
Drive drive = Drive();
Arm arm = Arm();

// rangefinder
Rangefinder rangefinder;
double distanceToBag = 5;  // how far away the robot should be from the bag when picking it up
int bagThreshold = 30;  // how far away the robot can be to detect a bag

// button for starting program
const int buttonPin = BOOT_FLAG_PIN;
bool upDown = false;

// output of the two line sensor pins
int reflectance1, reflectance2;

//state machine
enum ROBOT_STATES { LINE_FOLLOW_OUT, APPROACH_BAG, STREET_1, STREET_2, STREET_3, STREET_4, STREET_5, LINE_FOLLOW_CRUTCH, end };
int robotState;
int bagState = 0;  // 0 = Bag 1 (zone A), 1 = Bag 2 (zone B), 2 = Bag 3 (zone C)

// functions
void pickUpBag(void);
void dropOffBag(void);
void updateRobotState(void);

void setup() {
  Motor::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  Serial.begin(115200);
  drive.leftMotor.attach(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB);
  drive.rightMotor.attach(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB);
  rangefinder.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
  arm.armServo.attach(arm.servoPin);
  pinMode(drive.reflectancePin1, INPUT);
  pinMode(drive.reflectancePin2, INPUT);
  robotState = LINE_FOLLOW_OUT;
}

/**
 * Center robot on bag using rangefinder, approach bag, and pick up bag.
 **/
void pickUpBag(void) {
  double leftEdge, rightEdge, error;
  arm.moveTo(0);
  delay(200);
  rangefinder.getDistanceCM();
  delay(100);

  // while object is out of range, turn clockwise in place
  while (rangefinder.getDistanceCM() > bagThreshold) {
    drive.leftMotor.setSpeed(80);
    drive.rightMotor.setSpeed(-80);
  }
  leftEdge = drive.rightMotor.getCurrentDegrees(), rightEdge = leftEdge; // default for rightEdge causes no turning
  
  // wait for the object to get out of range
  while (rangefinder.getDistanceCM() < bagThreshold) {}
  rightEdge = drive.rightMotor.getCurrentDegrees();  // degrees when object is out of range
  
  drive.hardTurn((rightEdge - leftEdge) / 4);  // turn counterclockwise to center of bag

  // approach bag with proportional control
  while (rangefinder.getDistanceCM() > distanceToBag) {
    error = rangefinder.getDistanceCM();
    drive.leftMotor.setEffort(error * drive.kP / 4);  // divided by 2 on Dilce's bot
    drive.rightMotor.setEffort(error * drive.kP / 4);  // divided by 2 on Dilce's bot
  }

  arm.moveTo(0);
  drive.stop();
  drive.straight(-2);
  drive.hardTurn(170);
  drive.straight(-3.25);
  arm.moveTo(180);  // pick up bag
  delay(500);
}

/**
 * Deliver bag on its correct zone.
 **/
void dropOffBag(void) {
  drive.hardTurn(180);
  bagState++;
  if (bagState == 1) arm.moveTo(arm.deliverA);  // first bag delivered to zone A
  else if (bagState == 2) arm.moveTo(arm.deliverB);  // second bag delivered to zone B
  else if (bagState == 3) {
    arm.moveTo(180);
    arm.moveTo(arm.deliverC);  // third bag delivered to zone C
  }
  delay(1000);
}

/**
 * Switch statement to decide what the robot should do. 
 **/
void updateRobotState(void) {
  switch (robotState) {
    case LINE_FOLLOW_OUT:  // going down STREET_2 heading towards pick-up zone
      if ((reflectance1 >= drive.threshold) && (reflectance2 >= drive.threshold)) { // line sensor sees pick-up zone
        delay(50);
        drive.softTurn(-85);
        robotState = APPROACH_BAG;
      } else drive.lineFollow(reflectance1, reflectance2);
      break;

    case LINE_FOLLOW_CRUTCH:  // after bag drop-off, deciding how to turn to get back onto STREET_2
      if ((reflectance1 >= drive.threshold) && (reflectance2 >= drive.threshold)) {  // line sensor sees delivery zone intersection
        if (bagState == 1) {
          drive.softTurn(-85);
          robotState = LINE_FOLLOW_OUT;
        } else if (bagState == 2) {
          drive.straight(2);
          robotState = STREET_3;
        } else if (bagState == 3) {
          drive.stop();
          robotState = end;
        }
      } else drive.lineFollow(reflectance1, reflectance2);
      break;
    
    case APPROACH_BAG:  // approaching to pick up a bag
      if ((reflectance1 > drive.threshold) && (reflectance2 > drive.threshold)) {  // line sensor sees T at pick-up zone
        drive.straight(-5);
        drive.hardTurn(-30);
        arm.moveTo(0);
        pickUpBag();
        drive.straight(3);
        robotState = STREET_1;
      } else drive.lineFollow(reflectance1, reflectance2);
      break;

    case STREET_1:  // leaving pick-up zone
      if ((reflectance1 > drive.threshold) && (reflectance2 > drive.threshold)) {  // line sensor seees end of pick-up street
        drive.stop();
        delay(100);
        drive.softTurn(85);
        robotState = STREET_2;
      } else drive.lineFollow(reflectance1, reflectance2);
      break;

    case STREET_2:  // returning from STREET_1, deciding which street to deliver bag to
      if ((reflectance1 > drive.threshold) && (reflectance2 > drive.threshold)) {  // line sensor sees delivery zone intersection
        drive.stop();
        delay(100);
        if (bagState == 0){
          drive.softTurn(85);
          robotState = STREET_3;
        } else if (bagState == 1){
          drive.softTurn(-85);
          robotState = STREET_4;
        } else if (bagState == 2){
          drive.straight(2);
          robotState = STREET_5;
        }   
      } else drive.lineFollow(reflectance1, reflectance2);
      break;

    case STREET_3:  // first bag drop-off on zone A and pick up of free-range bag
      if ((reflectance1 > drive.threshold) && (reflectance2 > drive.threshold)) {  // line sensor sees T at zone A
        drive.stop();
        delay(100);
        if (bagState == 2) {  // for free-range bag
          drive.hardTurn(90);
          drive.straight(5);
          drive.hardTurn(-30);
          arm.moveTo(0);
          delay(200);
          pickUpBag();  // pick up free-range bag
          drive.hardTurn(-35);
          robotState = STREET_2;
        } else {
          dropOffBag();
          robotState = LINE_FOLLOW_CRUTCH;
        }
      } else drive.lineFollow(reflectance1, reflectance2);
      break;

    case STREET_4:  // second bag drop-off on zone B
      if ((reflectance1 > drive.threshold) && (reflectance2 > drive.threshold)) {  // line sensor sees T at zone B
        drive.stop();
        delay(100);
        dropOffBag();
        robotState = LINE_FOLLOW_CRUTCH;
      } else drive.lineFollow(reflectance1, reflectance2);
      break;

    case STREET_5:  // third bag (free-rage bag) drop-off on zone C
      if ((reflectance1 > drive.threshold) && (reflectance2 > drive.threshold)) {  // line sensor sees T at zone C
        drive.stop();
        delay(100);
        dropOffBag();
        robotState = LINE_FOLLOW_CRUTCH;
      } else drive.lineFollow(reflectance1, reflectance2);
      break;

    case end:  // exit program
      break;
  }
}

void loop() { 
  while(digitalRead(buttonPin)) {}  // wait for button press
  delay(500);

  while(true) {
   reflectance1 = analogRead(drive.reflectancePin1);  // update line sensor pin 1 value
   reflectance2 = analogRead(drive.reflectancePin2);  // update line sensor pin 2 value
   updateRobotState();  // enter switch statement
  }
}