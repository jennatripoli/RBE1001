#include <Arduino.h>
#include <RBE1001Lib.h>
//#include <ESP32Servo.h>

//for sensors
const int reflectancePin1=39;
const int reflectancePin2=36;
Rangefinder ultrasonic;
int bagApproachThreshold = 20;
int bagThreshold = 0;
double distanceToBag = 5.0;
int zoneApproachThreshold = 10;
int zoneThreshold = 3;
int distanceToB = 7;
double atStopPointLeft = 0;
double atStopPointRight = 0;
double facingCenterLeft = 0;
double facingCenterRight = 0;

//button
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

//for line following (calling the linefollowing sensor functions)
int reflectance1;
int reflectance2;
int threshold = 1000;
double kp = 0.032;

//for main control
int bagCount = 0;

//for servo arm
Servo lifter;
const int servoPin = 33;
int deliverA = 0;
int deliverB = 90;
int deliverC = 135;

//state machines
//enum ROBOT_STATES{LINE_FOLLOW_OUT, APPROACH_BAG, COLLECT_BAG, LINE_FOLLOW_BACK, INTERSECTION_BACK, APPROACH_ZONE, DELIVER_BAG, APPROACH_INTERSECTION, INTERSECTION_OUT, FREE_RANGE_COLLECT};
enum ROBOT_STATES{LINE_FOLLOW_OUT, APPROACH_BAG, STREET_1, STREET_2A,STREET_2B, STREET_3, STREET_4, STREET_5, LINE_FOLLOW_CRUTCH};
int robotState;

enum BAG_STATES{START, A, B, C};
int bagState;

//functions
void updateBagState(void);
void lineFollow(int reflectance1, int reflectance2);
void hardTurn(double angle, double diam3, double track2);
void softTurn(double angle, double diam3, double track2);
double ultrasonicRead ();
void deliverBag(void);
void findFreeBag(double ultrasonic_distance);
void straight(double distance, double wheelDiameter);

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
  bagState = START;
}

void lineFollow(int reflectance_1, int reflectance_2){ //line following function
  float error = reflectance_1 - reflectance_2;
  float effort = kp * error;
  right_motor.setSpeed(defaultSpeed+effort);
  left_motor.setSpeed(defaultSpeed-effort);
}

void hardTurn(double angle, double diam3, double track2){ //for navigation
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    double degreeMove = (angle*track2)/diam3;
    left_motor.startMoveFor(degreeMove, 120);
    right_motor.moveFor(-degreeMove, 120);
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
  }

void softTurn(double angle, double diam3, double track2){ //for navigation
    double degreeMove = (2*angle*track2)/diam3;
    if (angle >= 0){
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      left_motor.moveFor(degreeMove, 120);
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
    }else{ 
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      right_motor.moveFor(-degreeMove, 120);
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
    }
  }

void straight(double distance, double wheelDiameter) {
  double spin = (360*distance)/(wheelDiameter*PI);
  left_motor.startMoveFor(spin, 150);
  right_motor.moveFor(spin, 150);
}


void deliverBag(void){ //for bag delivery
  if (ultrasonicRead() > zoneThreshold){
     lineFollow (reflectance1, reflectance2);
       if (bagState == A){
          lifter.write(deliverA); //to place on the ground
          //back up
          left_motor.startMoveFor(-90, 120);
          right_motor.moveFor(-90, 120);
        }
       if (bagState == B){
          delay(100);
          lifter.write(deliverB); //to place on 4mm
          //back up
          left_motor.startMoveFor(-90, 120);
          right_motor.moveFor(-90, 120);
        }
      if (bagState == C){
          lifter.write(deliverC); //to place on 8mm
          //back up
          left_motor.startMoveFor(-90, 120);
          right_motor.moveFor(-90, 120);
        }
  }
}

void turnToObject(float distanceFromObject) {
  delay(200);
  lifter.write(0);
    while (ultrasonic.getDistanceCM() > distanceFromObject) { // while object is out of range
        left_motor.setEffort(0.2);
        right_motor.setEffort(-0.2);
    }
    double leftEdge = right_motor.getCurrentDegrees(), rightEdge = leftEdge; // default for rightEdge causes no turning
    while (ultrasonic.getDistanceCM() < distanceFromObject) {} // wait for the object to get out of range
    if (ultrasonic.getDistanceCM() > distanceFromObject) rightEdge = right_motor.getCurrentDegrees(); // when object is out of range
      hardTurn((rightEdge - leftEdge) / 4, diam, track); // turn ccw to center of object (average between the two edges)
    while (ultrasonic.getDistanceCM() > distanceToBag) {
    float error = ultrasonic.getDistanceCM();
    left_motor.setEffort(error * kp / 1.9);
    right_motor.setEffort(error * kp/2);
    }
    left_motor.setEffort(0);
    right_motor.setEffort(0);
    straight(-2, diam);
    hardTurn(170, diam, track);
    straight(-3, diam);
    lifter.write(180);
    delay(500);
    straight(3, diam);
    //hardTurn(15-(rightEdge + leftEdge)/2, diam, track);
}

void dropOffBag(){
  hardTurn(180, diam, track);
  if (bagState = (A)){
    lifter.write(deliverA);
  } else if (bagState = (B)){
    lifter.write(deliverB);
  } else if (bagState = (C)){
    lifter.write(deliverC);
  }
}

void pickupBag(){
  straight(-5,diam);
  hardTurn(-30, diam, track);
  bagThreshold = 30;
  turnToObject(bagThreshold);
}

void updateRobotState(void){
 
  switch (robotState){

  case LINE_FOLLOW_OUT:     
        if ((reflectance1 >= threshold) && (reflectance2 >= threshold)){ //when it sees pick up zone
          delay(200);
          atStopPointLeft = left_motor.getCurrentDegrees(); //save position for free-range finding
          atStopPointRight = right_motor.getCurrentDegrees();
          delay(100);
          softTurn(-85, diam, track);
          robotState = APPROACH_BAG;
        } else {
          lineFollow(reflectance1, reflectance2);
        }
        break;

  case LINE_FOLLOW_CRUTCH:
        if ((reflectance1 >= threshold) && (reflectance2 >= threshold)){
          if (bagState = (A)){
           softTurn(-85, diam, track);
           robotState = LINE_FOLLOW_OUT;
          } else if (bagState = (B)){
           softTurn(85, diam, track);
           robotState = LINE_FOLLOW_OUT;
          }
        }else{
           lineFollow(reflectance1, reflectance2);
         }  

        break;
  
  case APPROACH_BAG:     
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){
         pickupBag();
         robotState = STREET_1;
       }else{
         lineFollow(reflectance1, reflectance2);
       }
      break;

  case STREET_1:    
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){//////////////////////////PREDICTED PROBLEM
         left_motor.setSpeed(0);
         right_motor.setSpeed(0);
         delay(100);
         softTurn(85, diam, track);
         robotState = STREET_2A;
       }else{
         lineFollow(reflectance1, reflectance2);
       }
      break;

  case STREET_2A:    // from the quad
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){
         left_motor.setSpeed(0);
         right_motor.setSpeed(0);
         delay(100);
         bagCount ++;
         if (bagState = (A)){
           softTurn(85, diam, track);
           robotState = STREET_3;
         } else if (bagState = (B)){
           softTurn(-85, diam, track);
           robotState = STREET_4;
         } else if (bagState = (C)){
           straight(2, diam);
           robotState = STREET_5;
         } else {
           lifter.write(180); /////////Diagnosing problems
           lifter.write(90);  /////////Diagnosing problems
         }
         
         
       }else{
         lineFollow(reflectance1, reflectance2);
       }
      break;

  case STREET_2B:    //to the quad
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){
         left_motor.setSpeed(0);
         right_motor.setSpeed(0);
         delay(100);
         softTurn(85, diam, track);
         robotState = STREET_1;
       }else{
         lineFollow(reflectance1, reflectance2);
       }
      break;

  case STREET_3:    
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){//////////////////////////PREDICTED PROBLEM
         left_motor.setSpeed(0);
         right_motor.setSpeed(0);
         delay(100);
         dropOffBag();
         updateBagState();
         robotState = LINE_FOLLOW_CRUTCH;
       }else{
         lineFollow(reflectance1, reflectance2);
       }

      break;

    case STREET_4:
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){//////////////////////////PREDICTED PROBLEM
         left_motor.setSpeed(0);
         right_motor.setSpeed(0);

         delay(100);
         dropOffBag();
         updateBagState();
         robotState = LINE_FOLLOW_CRUTCH;
       }else{
         lineFollow(reflectance1, reflectance2);
       }
      break;

    case STREET_5:
      if ((reflectance1 > threshold) && (reflectance2 > threshold)){//////////////////////////PREDICTED PROBLEM
         left_motor.setSpeed(0);
         right_motor.setSpeed(0);
         lifter.write(90);
         delay(100);
         dropOffBag();
       }else{
         lineFollow(reflectance1, reflectance2);
       }

      break;
  }
}

void updateBagState(void){ //for bag-based control
  switch(bagState){
    case START:
      if (bagCount == 1){
        bagState = A;
      }
      break;

    case A:
       if (bagCount == 2){
          bagState = B;
       }
       break;

    case B:
        if (bagCount == 3){
        bagState = C;
        }
        break;
    case C:
    break;
  }
}

void loop() { 
 while(digitalRead(buttonPin)) {} //wait for button press
  delay (500);

  while(true){
   reflectance1=analogRead(reflectancePin1);
   reflectance2=analogRead(reflectancePin2);

   updateRobotState();
   updateBagState();
  }
}