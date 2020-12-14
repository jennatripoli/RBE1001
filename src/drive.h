#include <Arduino.h>
#include <RBE1001Lib.h>

class Drive {
    public:
        Motor leftMotor, rightMotor;
        double diam = 2.75;  // diameter of drive wheels, in inches
        double track = 5.875;  // distance between wheels, in inches
        double defaultSpeed = 200;  // default speed for driving, in degrees/sec
        double kP = 0.1;  // proportional constant for drive wheels (0.06 on Dilce's bot)
        int threshold = 1000;  // minimum line sensor value for the line to be seen
        const int reflectancePin1 = 39, reflectancePin2 = 36;  // line sensor pins
        void hardTurn(double angle);
        void softTurn(double angle);
        void straight(double distance);
        void stop(void);
        void lineFollow(int reflectance1, int reflectance2);
};