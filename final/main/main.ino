#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4BumpSensors.h>
#include <Pololu3piPlus32U4Buttons.h>
#include <Pololu3piPlus32U4Buzzer.h>
#include <Pololu3piPlus32U4Encoders.h>
#include <Pololu3piPlus32U4IMU.h>
#include <Pololu3piPlus32U4IMU_declaration.h>
#include <Pololu3piPlus32U4LCD.h>
#include <Pololu3piPlus32U4LineSensors.h>
#include <Pololu3piPlus32U4Motors.h>
#include <Pololu3piPlus32U4OLED.h>


#include <Servo.h>
#include "sonar.h"
#include "Pcontroller.h"
#include "PDcontroller.h" 



using namespace Pololu3piPlus32U4;

// Robot states
enum RobotState {
  WANDER,
  CELEBRATION,
  RETURN_HOME
};

// Global variables
RobotState currentState = WANDER;
int trashCount = 0;
bool atHome = false;
#define minOutput -100
#define maxOutput 100


// Sensor arrays
uint16_t lineSensorValues[5];
uint16_t lineDetectionValues[5];
bool isOnBlack = false;

// Wall following variables
double wallDist = 0;
const double WALL_DISTANCE = 10.0; // cm
const int baseSpeed = 100;
const int calibrationSpeed = 100;
#define kp_obs 0.8     // Proportional gain for obstacle avoidance
#define kd_obs 0.2 

// Robot components
LineSensors lineSensors;
Motors motors;
Servo servo;
Buzzer buzzer;
OLED display;
BumpSensors bumpSensors;
Sonar sonar(4);
PDcontroller pd_obs(kp_obs, kd_obs, minOutput, maxOutput);

// Constants
const int CELEBRATION_DURATION = 3000; // ms
const int SPIN_SPEED = 100; // PWM value
const int WANDER_SPEED = 100; // PWM value
const int MAX_TRASH = 3;

void setup() {
  Serial.begin(9600);
  

  bumpSensors.calibrate();
  display.clear();
  display.print("Trash: 0");
  
  // Calibrate sensors
  calibrateSensors();
}

void loop() {
  // State machine
  switch (currentState) {
    case WANDER:
      wanderState();
      break;
    case CELEBRATION:
      celebrationState();
      break;
    case RETURN_HOME:
      returnHomeState();
      break;
  }
}

void wanderState() {
  // Check for trash (using bump sensors as example)
  if (bumpSensors.read() != 0) {
    currentState = CELEBRATION;
    return;
  }

  // Check if we've found all trash
  if (trashCount >= MAX_TRASH) {
    currentState = RETURN_HOME;
    return;
  }

  // Gather sensor data and avoid obstacles
  lineSensors.read(lineSensorValues);
  
  // Simple line following with obstacle avoidance
  if (lineSensorValues[2] > 500) { // Center sensor sees line
    motors.setSpeeds(WANDER_SPEED, WANDER_SPEED);
  } else if (lineSensorValues[1] > 500) { // Left sensor sees line
    motors.setSpeeds(-WANDER_SPEED, WANDER_SPEED);
  } else if (lineSensorValues[3] > 500) { // Right sensor sees line
    motors.setSpeeds(WANDER_SPEED, -WANDER_SPEED);
  } else {
    // Random wandering if no line detected
    motors.setSpeeds(WANDER_SPEED, WANDER_SPEED);
  }
}

void celebrationState() {
  // Stop motors
  motors.setSpeeds(0, 0);

  // Spin 360 degrees
  motors.setSpeeds(SPIN_SPEED, -SPIN_SPEED);
  delay(CELEBRATION_DURATION);
  motors.setSpeeds(0, 0);

  // Beep
  buzzer.play("L16 cdegreg4");

  // Update display
  display.clear();
  display.print("Trash: ");
  display.print(trashCount);
  
  // Increment trash count and return to wander
  trashCount++;
  currentState = WANDER;
}

void returnHomeState() {
  // Check if we're at home (using line sensors as example)
  lineSensors.read(lineSensorValues);
  
  if (lineSensorValues[0] > 500 && lineSensorValues[4] > 500) { // Both outer sensors see line
    if (!atHome) {
      atHome = true;
      buzzer.play("L16 cdegreg4");
      display.clear();
      display.print("Home!");
    }
    motors.setSpeeds(0, 0);
    return;
  }

  // Follow line back to home
  if (lineSensorValues[2] > 500) { // Center sensor sees line
    motors.setSpeeds(WANDER_SPEED, WANDER_SPEED);
  } else if (lineSensorValues[1] > 500) { // Left sensor sees line
    motors.setSpeeds(-WANDER_SPEED, WANDER_SPEED);
  } else if (lineSensorValues[3] > 500) { // Right sensor sees line
    motors.setSpeeds(WANDER_SPEED, -WANDER_SPEED);
  } else {
    // Search for line if lost
    motors.setSpeeds(WANDER_SPEED/2, -WANDER_SPEED/2);
  }
}

void calibrateSensors() {
  Serial.println("Starting sensor calibration...");

  // Clear old values
  for (int i = 0; i < 5; i++) {
    lineSensorValues[i] = 0;
  }

  // --- Turn 90° LEFT ---
  Serial.println("Turning 90° LEFT...");
  motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
  delay(3500);
  motors.setSpeeds(0, 0);  
  Serial.println("Finished 90° LEFT turn. Reading sensors...");
  
  // Read and print sensor values at 90° left orientation
  lineSensors.read(lineSensorValues);
  for (int i = 0; i < 5; i++) {
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(lineSensorValues[i]);
  }

  // --- Return to CENTER from Left ---
  Serial.println("Returning to CENTER orientation...");
  motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
  delay(3500); 
  motors.setSpeeds(0, 0);  
  Serial.println("Returned to CENTER. Reading sensors...");

  lineSensors.read(lineSensorValues);
  for (int i = 0; i < 5; i++) {
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(lineSensorValues[i]);
  }

  // --- Turn 90° RIGHT from center ---
  Serial.println("Turning 90° RIGHT...");
  motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
  delay(3500);
  motors.setSpeeds(0, 0);
  Serial.println("Finished 90° RIGHT turn. Reading sensors...");

  lineSensors.read(lineSensorValues);
  for (int i = 0; i < 5; i++) {
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(lineSensorValues[i]);
  }

  // --- Return to CENTER from Right ---
  Serial.println("Returning to CENTER orientation...");
  motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
  delay(3500); 
  motors.setSpeeds(0, 0);
  Serial.println("Returned to CENTER. Reading sensors...");

  lineSensors.read(lineSensorValues);
  for (int i = 0; i < 5; i++) {
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(lineSensorValues[i]);
  }

  // Final stop
  motors.setSpeeds(0, 0);
  Serial.println("Calibration finished.");
  delay(1000);
}

void wallFollowing() {
  wallDist = sonar.readDist();
  
  // Calculate error from desired wall distance
  double error = wallDist - WALL_DISTANCE;
  
  // Get PD controller output
  double pdOutput = pd_obs.update(error, 0);
  
  // Set motor speeds
  int leftSpeed = baseSpeed + pdOutput;
  int rightSpeed = baseSpeed - pdOutput;
  
  motors.setSpeeds(leftSpeed, rightSpeed);
}

void detectBlackLine() {
  lineSensors.read(lineDetectionValues);
  const int blackThreshold = 1500;
  
  for (int i = 0; i < 5; i++) {
    if (lineDetectionValues[i] > blackThreshold) {
      isOnBlack = true;
      return;
    }
  }
  isOnBlack = false;
}