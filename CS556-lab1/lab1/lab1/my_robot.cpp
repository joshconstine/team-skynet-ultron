#include <Pololu3piPlus32U4.h>
#include "my_robot.h"
using namespace Pololu3piPlus32U4;

Motors motors;

MyRobot::MyRobot(float baseSpeed) {
  this->baseSpeed = baseSpeed;
}

short int MyRobot::convertSpeed(float speed) {
    // Convert speed from m/s to motor input (max speed 0.4 m/s corresponds to input 400)
    return static_cast<short int>((speed / 0.4) * 400);
}


void MyRobot::forward(float distance, float speed) {
    if (speed < 0) speed = this->baseSpeed;  // Use baseSpeed if no speed is provided
    short int motorSpeed = convertSpeed(speed);
    motors.setSpeeds(motorSpeed, motorSpeed);

    // Calculate time needed to travel the given distance
    float time = distance / speed;  
    delay(static_cast<int>(time * 1000));  // Convert seconds to milliseconds
    halt();
}


void MyRobot::backward(float distance, float speed) {
    if (speed < 0) speed = this->baseSpeed;
    short int motorSpeed = convertSpeed(speed);
    motors.setSpeeds(-motorSpeed, -motorSpeed);

    float time = distance / speed;
    delay(static_cast<int>(time * 1000));
    halt();
}


void MyRobot::turn_left(float duration, float speed) {
    if (speed < 0) speed = baseSpeed;
    short int motorSpeed = convertSpeed(speed);
    motors.setSpeeds(-motorSpeed, motorSpeed);

    delay(static_cast<int>(duration * 1000));
    halt();
}



void MyRobot::turn_right(float duration, float speed) {
    if (speed < 0) speed = baseSpeed;
    short int motorSpeed = convertSpeed(speed);
    motors.setSpeeds(motorSpeed, -motorSpeed);

    delay(static_cast<int>(duration * 1000));
    halt();
}


// the offset should be   0.1 < offset < 1 for how tight the turn should be.
void MyRobot::turn_left_and_forward(float duration, float speed,  double offset) {
    if (speed < 0) speed = baseSpeed;
    short int motorSpeed = convertSpeed(speed);
    motors.setSpeeds(motorSpeed * offset, motorSpeed);

    delay(static_cast<int>(duration * 1000));
    halt();
}



// the offset should be   0.1 < offset < 1 for how tight the turn should be.
void MyRobot::turn_right_and_forward(float duration, float speed,  double offset) {
    if (speed < 0) speed = baseSpeed;
    short int motorSpeed = convertSpeed(speed);
    motors.setSpeeds(motorSpeed, offset * motorSpeed);

    delay(static_cast<int>(duration * 1000));
    halt();
}





void MyRobot::halt() {
    motors.setSpeeds(0, 0);
    delay(500);  // Small delay to ensure the robot halts completely
}
