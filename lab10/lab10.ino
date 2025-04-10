#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4Buttons.h>
using namespace Pololu3piPlus32U4;
#include "particle_filter.h"
#include "odometry.h"
#include "Map.h"

ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;

#define PI 3.14159

#define lenOfMap 144
#define N_particles 25
#define move_noise  0.1
#define rotate_noise 0.5
#define ultra_noise 0.1

#define diaL 3.2 //define physical robot parameters
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75

Motors motorsP;
Encoders encoders;

Odometry odometry(diaL, diaR, w, nL, nR, gearRatio);
ParticleFilter particle(lenOfMap, N_particles, move_noise, rotate_noise, ultra_noise);

uint8_t iter =0;

//odometry
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x, y, theta;
float x_last = 0.0;
float y_last = 0.0;
float theta_last = 0.0;

void setup() {
  Serial.begin(9600);
  while (!Serial) continue;

  Map map = Map();

  //This is an example of how to estimate the distance to a wall for the given
  //map, assuming the robot is at (0, 0) and has heading PI
  float origin[2] = {0.5,0.5};
  float theta = PI;
  float closestDist = map.closest_distance(origin,theta);  

  Serial.println(closestDist);
  
}

void loop() {
  movement();

  //Get odometer readings  
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;   
  odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);

  //movement function
  //Propagate particles by using move_particles.
  //Parameters are change from current and past odometer values 
  //TODO: Put code under here 
  float dx = x - x_last;
  float dy = y - y_last;
  float dtheta = theta - theta_last;
  
  // Move particles based on odometry changes
  particle.move_particles(dx, dy, dtheta);

  //Measaure, estimation, and resample
  //Calculate particle's posterior probabilities, calculate estimated robot's position, and resample
  //TODO: Put code under here 
  // Take a measurement and update particle probabilities
  particle.measure();

  // Display all particle locations and estimated robot location on screen   
  //TODO: Put code under here 
  particle.print_particles();
  
    
  //save last odometer reading
  //TODO: Fill in "..."
  x_last = x;
  y_last = y;
  theta_last = theta;
    
  iter++;
    
  delay(1000); //for easier viewing of output
 
}

void movement(){
  // Remote-Controlling 
  if(buttonA.isPressed()){ //turn left
    //movement function here
    //TODO: Put code under here
    // Turn left 90 degrees
    motorsP.setLeftSpeed(-100);
    motorsP.setRightSpeed(100);
    delay(500); // Adjust this delay based on your robot's turning speed
    motorsP.setLeftSpeed(0);
    motorsP.setRightSpeed(0);

    Serial.print("Left pressed!\n");
  } else if(buttonB.isPressed()){ // drive forward
    //movement function here
    //TODO: Put code under here
    // Drive forward a short distance
    motorsP.setLeftSpeed(100);
    motorsP.setRightSpeed(100);
    delay(1000); // Adjust this delay based on your robot's speed
    motorsP.setLeftSpeed(0);
    motorsP.setRightSpeed(0);

    Serial.print("Forward pressed!\n");
  } else if(buttonC.isPressed()){ // turn right
    //movement function here
    //TODO: Put code under here
    // Turn right 90 degrees
    motorsP.setLeftSpeed(100);
    motorsP.setRightSpeed(-100);
    delay(500); // Adjust this delay based on your robot's turning speed
    motorsP.setLeftSpeed(0);
    motorsP.setRightSpeed(0);

    Serial.print("Right pressed!\n");
  }
 
}

void estimate_position() {
  float x_est = 0.0;
  float y_est = 0.0;
  float angle_est = 0.0;

  for (uint8_t i = 0; i < N_particles; i++) {
    x_est += particle.particles[i].probability * particle.particles[i].x;
    y_est += particle.particles[i].probability * particle.particles[i].y;
    angle_est += particle.particles[i].probability * particle.particles[i].angle;
  }

  // Normalize angle to [0, 2π]
  while (angle_est > 2 * PI) {
    angle_est -= 2 * PI;
  }
  while (angle_est < 0) {
    angle_est += 2 * PI;
  }

  Serial.print("Iteration: ");
  Serial.println(iter);
  Serial.print("Estimated Position: (");
  Serial.print(x_est);
  Serial.print(", ");
  Serial.print(y_est);
  Serial.print("), Angle: ");
  Serial.println(angle_est);

  for (uint8_t i = 0; i < N_particles; i++) {
    Serial.print("Particle ");
    Serial.print(i);
    Serial.print(": (");
    Serial.print(particle.particles[i].x);
    Serial.print(", ");
    Serial.print(particle.particles[i].y);
    Serial.print("), Angle: ");
    Serial.print(particle.particles[i].angle);
    Serial.print(", Probability: ");
    Serial.println(particle.particles[i].probability);
  }

  iter++;
}
