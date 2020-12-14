#include <Arduino.h>
#include <RBE1001Lib.h>

class Arm {
    public:
        Servo armServo;
        const int servoPin = 33;
        int deliverA = 0, deliverB = 75, deliverC = 130;  // angles to deliver bags on zones A, B, and C
        void moveTo(int angle);
};