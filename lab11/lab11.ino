#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4Buttons.h>
using namespace Pololu3piPlus32U4;
#include "particle_filter.h" //Uncomment after importing your particle filter files from Lab 10
#include "odometry.h"
#include "Map.h"

#define PI 3.14159

#define lenOfMap 144
#define N_particles 25
#define move_noise  0.01
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

Odometry odometry(diaL, diaR, w, nL, nR);
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
  
  // Estimate motion
  float dx, dy, dtheta = x - x_last, y - y_last, theta - theta_last;
  particle.move_particles(dx, dy, dtheta);


  //Measaure, estimation, and resample
  //Calculate particle's posterior probabilities, calculate estimated robot's position, and resample
  //TODO: Put code under here 
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

//Movement Algorithm
//TODO: Implement a movement algorithm that will allow your robot to autonomously move throughout the field. 
void movement(){
  float front_dist = particle._sonar.readDist();

  if (front_dist < 24) {
    // Turn in place to the left
    motorsP.setSpeeds(-50, 50);
    delay(500);
    motorsP.setSpeeds(0, 0);
  } else {
    // Move forward
    motorsP.setSpeeds(50, 50);
    delay(500);
    motorsP.setSpeeds(0, 0);
  }
}
