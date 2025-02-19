#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "printOLED.h"
#include "odometry.h"
using namespace Pololu3piPlus32U4;
#include "my_robot.h"
Motors motors;
Encoders encoders;


/*DEFINE VARIBLES FOR PHYSICAL ROBOT PARAMETRS 
BY INSERTING THE PARAMETER VALUES FROM THE ROBOT DOCUMENTATION*/
#define DEAD_RECKONING false // if "false": uses formula from the lecture slides to calculate theta. if "true", uses IMU data to calculate theta.
#define diaL 3.2 //define physical robot parameters (cm)
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6 //cm
#define gearRatio 75

#define BaseSpeed 100  // unit: mm per second

MyRobot robot; // 


Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING);


int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;

float x = 0.0, y = 0.0, theta = 0.0;

void setup() {
  Serial.begin(9600);
  delay(20);
}

void loop() {  

      //(TASK 2.1) Test Encoders while Staying Still 

      //IMPORT FUNCTIONS FROM LAB1 (MOVE FORWARD, TURN LEFT, ...) 
      //TO ENABLE ROBOT TO MOVE ONE METER FORWARD, CLOCKWISE AND COUNTERCLOCKWISE

      //Encoder code should always be before updating and movement
      // Read data from encoders
      // deltaL = encoders.getCountsAndResetLeft();
      // deltaR = encoders.getCountsAndResetRight();

      // // Increment total encoder count
      // encCountsLeft += deltaL;
      // encCountsRight += deltaR;

      // Serial.println("Left: ");
      // Serial.println(encCountsLeft);

      // Serial.println("Right: ");
      // Serial.println(encCountsRight);

      //(TASK 2.2) TEST ENCODERS WHILE MOVING EACH OF THE THREE MOVEMENTS LISTED BELOW (ONE AT A TIME) 
      //(NOTE: YOU SHOULD UPDATE AND USE PRIMITIVE FUNCTIONS FROM LAB1)
      //DO NOT DELETE CODE AFTER EACH TASK, COMMENT OUT INSTEAD
      
      //(2.2a) MOVE FORWARD ON A STRAIGHT LINE FOR ONE METER
      // robot.forward(1.0, 0.1);

      // deltaL = encoders.getCountsAndResetLeft();
      // deltaR = encoders.getCountsAndResetRight();

      // // Increment total encoder count
      // encCountsLeft += deltaL;
      // encCountsRight += deltaR;

      // Serial.println("Left: ");
      // Serial.println(encCountsLeft);

      // Serial.println("Right: ");
      // Serial.println(encCountsRight);
      // delay(50000);

      //(2.2b) MOVE BACKWARD ON A STRAIGHT LINE FOR ONE METER

      // deltaL = encoders.getCountsAndResetLeft();
      // deltaR = encoders.getCountsAndResetRight();

      // robot.backward(1.0, 0.1);
      // deltaL = encoders.getCountsAndResetLeft();
      // deltaR = encoders.getCountsAndResetRight();

      // // Increment total encoder count
      // encCountsLeft += deltaL;
      // encCountsRight += deltaR;

      // Serial.println("Left: ");
      // Serial.println(encCountsLeft);

      // Serial.println("Right: ");
      // Serial.println(encCountsRight);
      // delay(500000);

      //(2.2c) TURN LEFT FOR 90 DEGREES
      // deltaL = encoders.getCountsAndResetLeft();
      // deltaR = encoders.getCountsAndResetRight();
      // robot.turn_left(1.0, .085);

      // deltaL = encoders.getCountsAndResetLeft();
      // deltaR = encoders.getCountsAndResetRight();

      // encCountsLeft += deltaL;
      // encCountsRight += deltaR;

      // Serial.println("Left: ");
      // Serial.println(encCountsLeft);

      // Serial.println("Right: ");
      // Serial.println(encCountsRight);
      // delay(500000);


      // PRINT THE LEFT AND RIGHT ODOMETRY VALUES ON OLED

      // PRINT THE LEFT AND RIGHT ODOMETRY VALUES ON SERIAL MONITOR


      //(TASK 3.1) IMPLEMENT ODOMETRY 

      /*UNCOMMENT Odometry.update_odom DOWN BELOW*/
      odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta); //calculate robot's position

      /*NOW GO TO odometry.cpp file AND COMPLETE THE udate_odom FUNCTION. 
      IN odometry.cpp, ADD CODES TO ENABLE THE ROBOT TO 
      TO CALCULATE x, y, theta, AND
      TO PRINT THE CALCULATED VALUES TO OLED SCREEN AND SERIAL MONITOR*/
      

      //TEST ODOMETRY WHILE MOCING EACH OF THE THREE MOVEMENTS LISTED BELOW (ONE AT A TIME)
      //DO NOT DELETE CODE AFTER EACH TASK, COMMENT OUT INSTEAD

      //(3.2a)  15-meter straight line down the hallway

      //(3.2c)  1-meter square clockwise
      robot.forward(1.0, 0.1);
      odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);
      robot.turn_left(1.0, 0.085);
      robot.forward(1.0, 0.1);
      robot.turn_left(1.0, 0.085);
      robot.forward(1.0, 0.1);
      odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);
      robot.turn_left(1.0, 0.085);
      robot.forward(1.0, 0.1);
       
      robot.turn_left(1.0, 0.085);
      odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);

      delay(100000000);
      //(3.3e)  1-meter square counterclockwise
}
