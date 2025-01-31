#include <Pololu3piPlus32U4.h>
#include "my_robot.h"
using namespace Pololu3piPlus32U4;

// Motors motors;  // Define the motors object (for the normal way)

MyRobot robot;  // Instantiate the MyRobot class with the default base speed


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

void loop(){
    //Sets the speeds for motors 
    //The value of the input is given in mm/s with a maximum value of 400
    
    //move forward
    // motors.setSpeeds(400, 400);
    delay(10000);  // Move forward for 2 seconds

    robot.forward(1.0);

    // delay(2000); 
    robot.backward(1.0);

    //turn right(in place)
    // motors.setSpeeds(400, -400);
    delay(1000);  // Turn right for 1 second
    robot.turn_right(2.0); // Turn right in place for 2 seconds at base speed
    // delay(2000); 

    //turn left (in place)
    // motors.setSpeeds(-400, 400);
    delay(1000);  // Turn left for 1 second
    robot.turn_left(2.0); // Turn left in place for 2 seconds at base speed
      // delay(2000); 

    //move forward while turning right
    // motors.setSpeeds(400, 200);
    delay(2000);  // Move and turn right for 2 seconds
    // robot.forward(0.5, 0.2); // Move forward 0.5 meters with a custom speed of 0.2 m/s

    //move forward while turning lft
    // motors.setSpeeds(200, 400);
    // delay(2000);  // Move and turn left for 2 seconds
    delay(2000); 
    //move backwards
    // motors.setSpeeds(-400, -400);
    // delay(2000);  // Move backward for 2 seconds
    robot.backward(0.5); // Move backward 1 meter at base speed


    //halt
    // motors.setSpeeds(0, 0);
    robot.halt();

    robot.turn_left_and_forward(.5, 1, .8);

    delay(500);  // Halt for 5 seconds   
    robot.turn_left_and_forward(.5, 1, .6);

    delay(500);
    robot.turn_left_and_forward(.5, 1, .3);

    delay(500);
    robot.turn_left_and_forward(.5, 1, .1);

    delay(500);

    
    robot.turn_right_and_forward(.5, 1, .8);

    delay(500);  // Halt for 5 seconds   
     robot.turn_right_and_forward(.5, 1, .6);

    delay(500);
    robot.turn_right_and_forward(.5, 1, .3);

    delay(500);
    robot.turn_right_and_forward(.5, 1, .1);

    delay(500);
}
