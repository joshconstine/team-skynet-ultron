#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "sonar.h"
#include "Pcontroller.h"
#include "PDcontroller.h"
using namespace Pololu3piPlus32U4;

//Odometry Parameters
#define diaL 3.2
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75

//Update kp and kd based on your testing
#define minOutput -100
#define maxOutput 100
#define kp 55
#define kd 10
#define base_speed 300

Motors motors;
Servo servo;
Sonar sonar(4);

PDcontroller PDcontroller(kp, kd, minOutput, maxOutput);

const double distFromWall=10.0; // Goal distance from wall (cm)


double wallDist;
double PDout;
int leftSpeed;
int rightSpeed;

void setup() {
  Serial.begin(9600);
  servo.attach(5);
  delay(40);
  //Move Sonar to desired direction using Servo
    delay(40);
  //Move Sonar to desired direction using Servo
  servo.write(180);
}

void loop() {
  //DO NOTE DELETE CODE AFTER EACH TASK, COMMENT OUT INSTEAD
  wallDist = sonar.readDist();


  //UNCOMMENT AFTER IMPLEMENTING PDcontroller
  PDout = PDcontroller.update(wallDist, distFromWall); //uncomment if using PDcontroller 
  leftSpeed = base_speed + PDout;
  rightSpeed = base_speed - PDout;
  //(LAB 5 - TASK 3.1) IMPLEMENT PDCONTROLLER 
  
  
  motors.setSpeeds(leftSpeed, rightSpeed);

  // Debugging (print values to Serial Monitor)
  Serial.print("Wall Distance: ");
  Serial.print(wallDist);
  Serial.print(" | PDout: ");
  Serial.print(PDout);
  Serial.print(" | Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightSpeed);

  delay(100);
  /*FIRST GO TO PDcontroller.h AND ADD PRIVATE VARIABLES NEEDED.
    THEN GO TO PDcontroller.cpp AND COMPLETE THE update FUNCTION.
    ONCE YOU IMPLEMENT update, UNCOMMENT CODE ABOVE TO USE CONTROLLER.*/

  //(LAB 5 - TASK 3.2) PDCONTROLLER WALL FOLLOWING

  /*NOW THAT YOU HAVE IMPLEMENTED PDCONTROLLER, TAKE THE OUTPUT FROM PDout
  AND SET THE MOTOR SPEEDS. CHANGE THE KP, KD, AND CLAMPING VALUES AT THE TOP
  TO TEST (B-D).
  Hint: Also use baseSpeed when setting motor speeds*/

  //Also print outputs to serial monitor for testing purposes

}
