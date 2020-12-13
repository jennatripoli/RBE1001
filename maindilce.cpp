#include <Arduino.h>
#include <RBE1001Lib.h>

//for rangefinder
Rangefinder ultrasonic;
int bagThreshold = 20;
double distanceToBag = 3.0;
int zoneThreshold = 3;
double leftEdge = 0;
double rightEdge = 0;

//button for starting program
const int buttonPin = BOOT_FLAG_PIN;
bool upDown=false;

//for driving
Motor left_motor;
Motor right_motor;
double diam = 2.75;
double track = 5.875;
int defaultSpeed = 150;
double distanceCorrection = 0.95;
double bagDistance;
double zoneDistance;

//for line following
const int reflectancePin1=39;
const int reflectancePin2=36;
int reflectance1;
int reflectance2;
int threshold = 1100;
double kp = 0.05;

//for servo arm
Servo lifter;
const int servoPin = 33;
int deliverA = -35; // bag 1
int deliverB = 45; // bag 2
int deliverC = 100; // bag 3

//state machine
enum ROBOT_STATES{LINE_FOLLOW_OUT, APPROACH_BAG, STREET_1, STREET_2, STREET_3, STREET_4, STREET_5, LINE_FOLLOW_CRUTCH, end};
int robotState;
int bagState = 0; // 0 = STREET_3, 1 = STREET_4, 2 = STREET_5

//functions
void lineFollow(int reflectance1, int reflectance2);
void hardTurn(double angle);
void softTurn(double angle);
double ultrasonicRead();
void deliverBag(void);
void turnToObject(float);
void straight(double distance);

void setup() {
  Motor::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  Serial.begin(115200);
  // pin definitions https://wpiroboticsengineering.github.io/RBE1001Lib/RBE1001Lib_8h.html#define-members
  left_motor.attach(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB);
  right_motor.attach(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB);
  ultrasonic.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
  lifter.attach(servoPin);
  pinMode(reflectancePin1,INPUT);
  pinMode(reflectancePin2,INPUT);
  robotState = LINE_FOLLOW_OUT;
}

void lineFollow(int reflectance_1 , int reflectance_2){ //line following function
  float error = reflectance_1 - reflectance_2;
  float effort = kp * error;
  right_motor.setSpeed(defaultSpeed+effort);
  left_motor.setSpeed(defaultSpeed-effort);
}

void hardTurn(double angle){ //for navigation
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    double degreeMove = (angle*track)/diam;
    left_motor.startMoveFor(degreeMove, 100);
    right_motor.moveFor(-degreeMove, 100);
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
  }

void softTurn(double angle){ //for navigation   
    double degreeMove = (2*angle*track)/diam;
    if (angle >= 0){
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      left_motor.moveFor(degreeMove, 100);
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
    }else{ 
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      right_motor.moveFor(-degreeMove, 100);
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
    }
  }

void straight(double distance) { // lets robot travel in a straight line
  double spin = (360*distance)/(diam*PI);
  left_motor.startMoveFor(spin, defaultSpeed);
  right_motor.moveFor(spin, defaultSpeed);
}


void deliverBag(void){ //for bag delivery
  if (ultrasonicRead() > zoneThreshold){
     lineFollow (reflectance1, reflectance2);
       if (bagState == 1){
          lifter.write(deliverA); //to place on the ground         
          left_motor.startMoveFor(-90, 120);//back up
          right_motor.moveFor(-90, 120);
        }
       if (bagState == 2){
          delay(100);
          lifter.write(deliverB); //to place on 4mm
          left_motor.startMoveFor(-90, 120);//back up
          right_motor.moveFor(-90, 120);
        }
      if (bagState == 3){
          lifter.write(deliverC); //to place on 8mm
          left_motor.startMoveFor(-90, 120); //back up
          right_motor.moveFor(-90, 120);
        }
  }
}

void turnToObject(float distanceFromObject) { //Uses rangefinder to locate and pickup bag
  delay(100);
  leftEdge = 0;
  rightEdge = 0;
  lifter.write(0);
    while (ultrasonic.getDistanceCM() > distanceFromObject) { // while object is out of range
        left_motor.setEffort(0.2);
        right_motor.setEffort(-0.2);
    }
    leftEdge = right_motor.getCurrentDegrees(); //, rightEdge = leftEdge; // default for rightEdge causes no turning
    while (ultrasonic.getDistanceCM() < distanceFromObject) {} // wait for the object to get out of range
    if (ultrasonic.getDistanceCM() > distanceFromObject) rightEdge = right_motor.getCurrentDegrees(); // when object is out of range
    hardTurn((rightEdge - leftEdge) / 4); // turn ccw to center of object (average between the two edges)
    while (ultrasonic.getDistanceCM() > distanceToBag) {
    float error = ultrasonic.getDistanceCM();
    left_motor.setEffort(error * kp / 2);
    right_motor.setEffort(error * kp / 2);
    }
    left_motor.setEffort(0);
    right_motor.setEffort(0);
    straight(-2);
    hardTurn(170);
    straight(-3);
    lifter.write(180);
    delay(500);
    straight(3);
}

void dropOffBag(){ //Drops the bags off
  hardTurn(180);
  bagState ++;
  if (bagState == 1){
    lifter.write(deliverA);
  } else if (bagState == 2){
    lifter.write(deliverB);
  } else if (bagState == 3){
    lifter.write(deliverC);
  }
  delay(500);
}

void updateRobotState(void){
 
  switch (robotState){

  case LINE_FOLLOW_OUT:  // Robot goes down STREET_2 heading towards the Bag Pick Up area
        if ((reflectance1 >= threshold) && (reflectance2 >= threshold)){ //when it sees pick up zone
          delay(50);
          softTurn(-85);
          robotState = APPROACH_BAG;
        } else {
          lineFollow(reflectance1, reflectance2);
        }
        break;

  case LINE_FOLLOW_CRUTCH: // After Robot drops off bags. Decides which way to turn to get back to STREET_2
        if ((reflectance1 >= threshold) && (reflectance2 >= threshold)){
          if (bagState == 1){
           softTurn(-85);
           robotState = LINE_FOLLOW_OUT;
          } else if (bagState == 2){
           straight(2);
           robotState = STREET_3;
          } else if (bagState == 3){
           left_motor.setSpeed(0);
           right_motor.setSpeed(0);
           robotState = end;
          }
        }else{
           lineFollow(reflectance1, reflectance2);
         }  

        break;
  
  case APPROACH_BAG:    // Approaches to pick up bag
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){
         straight(-5);
         hardTurn(-30);
         turnToObject(bagThreshold);
         robotState = STREET_1;
       }else{
         lineFollow(reflectance1, reflectance2);
       }
      break;

  case STREET_1:    //Robot leaving Bag Pick Up area
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){
         left_motor.setSpeed(0);
         right_motor.setSpeed(0);
         delay(100);
         softTurn(85);
         robotState = STREET_2;
       }else{
         lineFollow(reflectance1, reflectance2);
       }
      break;

  case STREET_2:  // Robot returning from the STREET_1 and deciding which street to put bags down
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){
         left_motor.setSpeed(0);
         right_motor.setSpeed(0);
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
       }else{
         lineFollow(reflectance1, reflectance2);
       }
      break;

  case STREET_3:   // First Bag drop off
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){
         left_motor.setSpeed(0);
         right_motor.setSpeed(0);
         delay(100);
         if (bagState == 2 ){
          hardTurn(90);
          straight(5);
          hardTurn(-30);
          delay(100);
          turnToObject(bagThreshold);
          delay(100);
          hardTurn(-45);
          robotState = STREET_2;
         } else {
          dropOffBag();
          robotState = LINE_FOLLOW_CRUTCH;
         }
       }else{
         lineFollow(reflectance1, reflectance2);
       }
      break;

    case STREET_4:  // Second Bag drop off
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){
         left_motor.setSpeed(0);
         right_motor.setSpeed(0);
         delay(100);
         dropOffBag();
         robotState = LINE_FOLLOW_CRUTCH;
       }else{
         lineFollow(reflectance1, reflectance2);
       }
      break;

    case STREET_5:  // Third Bag drop off
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){
         left_motor.setSpeed(0);
         right_motor.setSpeed(0);
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