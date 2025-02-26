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
#define kp 45
#define kd 2
#define base_speed 90

Motors motors;
Servo servo;
Sonar sonar(4);

PDcontroller PDcontroller(kp, kd, minOutput, maxOutput);

const double distFromWall=10.0; // Goal distance from wall (cm)

double wallDist;

void setup() {
  Serial.begin(9600);
  servo.attach(5);
  delay(40);
  //Move Sonar to desired direction using Servo
}

void loop() {
  //DO NOTE DELETE CODE AFTER EACH TASK, COMMENT OUT INSTEAD
  wallDist = sonar.readDist();


  //UNCOMMENT AFTER IMPLEMENTING PDcontroller
  PDout = PDcontroller.update(wallDist, distFromWall); //uncomment if using PDcontroller 

  //(LAB 5 - TASK 3.1) IMPLEMENT PDCONTROLLER 
  
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
