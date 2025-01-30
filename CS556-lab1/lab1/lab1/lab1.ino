#include <Pololu3piPlus32U4.h>
#include "my_robot.h"
using namespace Pololu3piPlus32U4;

void setup() {
  Serial.begin(9600);
  delay(15);
}
// or this task, we will be using the setSpeeds function, which requires two short int inputs
// (leftSpeed and rightSpeed). These inputs are signed values with positive being forward, negative being
// backward. The value of the input is given in mm/s with a maximum value of 400 (because the maximum
// speed of the robot is 0.4 m/s).
// 6a on Task Sheet: Test your understanding by implementing simple primitives: move forward, turn left (in
// place), turn right (in place), move forward while turning, move backwards, and halt. You will only need to
// edit lab1.ino, but feel free to browse the other files for documentation. Record the robot moving in all
// directions that were listed and have the TAs check that the robot moves correctly.
// void loop(){
    //Sets the speeds for motors 
    //The value of the input is given in mm/s with a maximum value of 400
    
    //move forward

    //turn right(in place)

    //turn left (in place)

    //move forward while turning right

    //move forward while turning lrft

    //move backwards

    //halt
}
