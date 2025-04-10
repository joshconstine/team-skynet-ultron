#include <Pololu3piPlus32U4.h>
#include <Gaussian.h>
#include "particle_filter.h"
#include "sonar.h"
using namespace Pololu3piPlus32U4;

/* Initialize particle filter and particle list */
ParticleFilter::ParticleFilter(int lenOfMap, int num_particles, float translation_variance, float rotation_variance, float measurement_variance) {
  //TODO: set class variables to passed variables
  _lenOfMap = lenOfMap;
  _num_particles = num_particles;
  _translation_variance = translation_variance;
  _rotation_variance = rotation_variance;
  _measurement_variance = measurement_variance;

  _iter = 0; //Iterator

  _x_est = 0.0;
  _y_est = 0.0;
  _angle_est = 0.0;

  Map _mp = Map();

  //generate particles with random location and rotation using _lenOfMap for x, y, and angle.
  //All probabilities will be the same at the start.
  //TODO: fill in "..."
  for(uint8_t i=0; i<_num_particles; i++){
    _particle_list[i].x = random(_lenOfMap);
    _particle_list[i].y = random(_lenOfMap);
    _particle_list[i].angle = random(360) * PI / 180.0; // Random angle in radians
    _particle_list[i].probability = 1.0 / _num_particles; // Equal probability initially
  }
}

/* Propagate motion of particles */
void ParticleFilter::move_particles(float dx, float dy, float dtheta){
//Apply motion to each particle by using the Gaussian class.
//You will need to utilize _translation_variance and _rotation_variance
//TODO: Put code under here
Gaussian translationGaussian = Gaussian(0, _translation_variance);
Gaussian rotationGaussian = Gaussian(0, _rotation_variance);

//Then add the current _particle_list[i] angle value with dtheta + random rotational
//And add the current _particle_list[i] x value with dx + random translation times the appropriate sin/cos angle value
//Do the same for y
//Consider using a for loop
//TODO: Put code under here
for(uint8_t i=0; i<_num_particles; i++) {
  // Add noise to rotation
  float noisy_dtheta = dtheta + rotationGaussian.random();
  
  // Update particle angle
  _particle_list[i].angle += noisy_dtheta;
  
  // Add noise to translation
  float noisy_dx = dx + translationGaussian.random();
  float noisy_dy = dy + translationGaussian.random();
  
  // Update particle position
  _particle_list[i].x += noisy_dx;
  _particle_list[i].y += noisy_dy;
  
  // Keep particles within map boundaries
  if (_particle_list[i].x < 0) _particle_list[i].x = 0;
  if (_particle_list[i].x > _lenOfMap) _particle_list[i].x = _lenOfMap;
  if (_particle_list[i].y < 0) _particle_list[i].y = 0;
  if (_particle_list[i].y > _lenOfMap) _particle_list[i].y = _lenOfMap;
  
  // Normalize angle to [0, 2*PI]
  while (_particle_list[i].angle > 2*PI) _particle_list[i].angle -= 2*PI;
  while (_particle_list[i].angle < 0) _particle_list[i].angle += 2*PI;
}
}

/* Calculate particle posterior probabilities*/
void ParticleFilter::measure(){
  float norm_factor=0;
  float maxprob=-99;
  
  // Get the actual sensor reading
  float actual_distance = _sonar.read();
  
  //Put next chunk in for loop:
  for(uint8_t i=0; i<_num_particles; i++) {
    // compute what the distance should be, if particle position is accurate
    /*If the robot uses closest_distance to calculate posterior probabilities in a particle filter,
      the return of this function helps assess the likelihood of the particle's position given the map layout.
      
      For example, if a particle's estimated sensor reading (distance to nearest obstacle) 
      aligns with the robot's actual sensor reading, 
      this particle is likely to be close to the robot's true position.*/
      //TODO: Fill in "..."
      float origin[2] = {_particle_list[i].x, _particle_list[i].y};
      float particleDist = _mp.closest_distance(origin, _particle_list[i].angle);

      // compute the probability P[measured z | robot @ x]
      //TODO: Put code under here
      Gaussian measurementGaussian = Gaussian(particleDist, _measurement_variance);
      float likelihood = measurementGaussian.plot(actual_distance);
      
      // Use log probabilities to avoid numerical instability
      float log_likelihood = log(likelihood);

      /* compute the probability P[robot@x | measured]
     NOTE: This is not a probability, since we don't know P[measured z]
     Hence we normalize afterwards */
     //TODO: Put code under here
     _particle_list[i].probability = log_likelihood;
     
     // Keep track of maximum probability for resampling
     if (log_likelihood > maxprob) {
       maxprob = log_likelihood;
     }

     // normalize probabilities (take P[measured z into account])
     //TODO: Put code under here
     // We'll normalize after the loop
  }
  //End of for loop

  //take each probability and normalize by norm_factor (in a for loop)
  //TODO: Put code under here
  // Convert from log probabilities to probabilities and normalize
  float sum_prob = 0.0;
  for(uint8_t i=0; i<_num_particles; i++) {
    _particle_list[i].probability = exp(_particle_list[i].probability - maxprob);
    sum_prob += _particle_list[i].probability;
  }
  
  // Normalize probabilities
  for(uint8_t i=0; i<_num_particles; i++) {
    _particle_list[i].probability /= sum_prob;
  }

  //Outside all for loops, call resample(maxprob) at the end of function
  //TODO: Put code under here
  resample(maxprob);
}

/* Resample particles */
void ParticleFilter::resample(float maxprob){
  float b = 0.0;
  float norm_tot=0;
  uint8_t maxind=0;
  uint8_t index = (int)random(_num_particles);

  Particle temp_particles[_num_particles];

  //roulette wheel selection of particles based on probabilities
  //This should be in a for loop
  //TODO: Put code under here
  for(uint8_t i=0; i<_num_particles; i++) {
    // Roulette wheel selection
    b = random(1000) / 1000.0; // Random value between 0 and 1
    float sum = 0.0;
    
    for(uint8_t j=0; j<_num_particles; j++) {
      sum += _particle_list[j].probability;
      if (sum >= b) {
        temp_particles[i] = _particle_list[j];
        break;
      }
    }
  }
  //End of for loop

  //take each temp particle probability and normalize by norm_tot
  //This should be in a for loop
  //TODO: Put code under here
  norm_tot = 0.0;
  for(uint8_t i=0; i<_num_particles; i++) {
    norm_tot += temp_particles[i].probability;
  }
  
  for(uint8_t i=0; i<_num_particles; i++) {
    temp_particles[i].probability /= norm_tot;
  }
  //End of for loop

  //Set each particle in _particle_list to temp_particles
  //This should be in a for loop
  //TODO: Put code under here
  for(uint8_t i=0; i<_num_particles; i++) {
    _particle_list[i] = temp_particles[i];
  }
  //End of for loop
}

/* Print particle position and probabilities to serial monitor */
void ParticleFilter::print_particles(){
  //print current iteration number, particles, and estimated location
  //Before estimating position, call the estimate_position function.
  //TODO: Put code under here
  estimate_position();
  
  Serial.print("Iteration: ");
  Serial.println(_iter);
  
  Serial.print("Estimated Position: (");
  Serial.print(_x_est);
  Serial.print(", ");
  Serial.print(_y_est);
  Serial.print("), Angle: ");
  Serial.println(_angle_est * 180.0 / PI);
  
  Serial.println("Particles:");
  for(uint8_t i=0; i<_num_particles; i++) {
    Serial.print("Particle ");
    Serial.print(i);
    Serial.print(": (");
    Serial.print(_particle_list[i].x);
    Serial.print(", ");
    Serial.print(_particle_list[i].y);
    Serial.print("), Angle: ");
    Serial.print(_particle_list[i].angle * 180.0 / PI);
    Serial.print(", Probability: ");
    Serial.println(_particle_list[i].probability);
  }
  
  _iter++;
}

/* Estimate the position of the robot using posterior probabilities */
void ParticleFilter::estimate_position(){
  _x_est = 0.0;
  _y_est = 0.0;
  _angle_est = 0.0;

  //Add current _x_est to particle list probability times particle list x
  //Do the same for y and angle
  //This should be in a for loop
  //TODO: Put code under here
  for(uint8_t i=0; i<_num_particles; i++) {
    _x_est += _particle_list[i].probability * _particle_list[i].x;
    _y_est += _particle_list[i].probability * _particle_list[i].y;
    _angle_est += _particle_list[i].probability * _particle_list[i].angle;
  }
  //End of for loop
  
  // Normalize angle to [0, 2*PI]
  while (_angle_est > 2*PI) _angle_est -= 2*PI;
  while (_angle_est < 0) _angle_est += 2*PI;
}
