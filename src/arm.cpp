#include <arm.h>

/**
 * Move arm to an angle.
 * @param angle  where to move the arm to, in degrees (0 = horizontal)
 **/
void Arm::moveTo(int angle) {
    armServo.write(angle);
}