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

  _x_est = 0;
  _y_est = 0;
  _angle_est = 0;

  _mp = Map();

  //generate particles with random location and rotation using _lenOfMap for x, y, and angle.
  //All probabilities will be the same at the start.
  //TODO: fill in "..."
  for(uint8_t i=0;i<_num_particles; i++){
    
  float rx = (float)random(0, _lenOfMap);  // e.g., 0..144
  float ry = (float)random(0, _lenOfMap);
  float rAngleDeg = (float)random(-180,180); // -180..180
  float rAngle = rAngleDeg * PI / 180.0;     // convert to radians
  


  _particle_list[i].x = rx;
  _particle_list[i].y = ry;
  _particle_list[i].angle = rAngle;
  _particle_list[i].probability = 1.0/_num_particles;
  }

}

/* Propagate motion of particles */
void ParticleFilter::move_particles(float dx, float dy, float dtheta){
  //Apply motion to each particle by using the Gaussian class.
  //You will need to utilize _translation_variance and _rotation_variance
  //TODO: Put code under here

  Gaussian gTrans(0, _translation_variance * _translation_variance);
  Gaussian gRot(0, _rotation_variance * _rotation_variance);

  //Then add the current _particle_list[i] angle value with dtheta + random rotational
  //And add the current _particle_list[i] x value with dx + random translation times the appropriate sin/cos angle value
  //Do the same for y
  //Consider using a for loop
  //TODO: Put code under here

  for(uint8_t i=0; i<_num_particles; i++){
      // Add noisy rotation
      float noisyRot = dtheta + gRot.random();  // random ~ Normal(0,rotate_variance)

      // We'll rotate the (dx,dy) by the *particle's current angle* 
      // then add noise in the translation
      // If you prefer just forward distance instead of dx/dy, adapt accordingly.

      float noisyX = dx + gTrans.random();
      float noisyY = dy + gTrans.random();

      // Update angle
      _particle_list[i].angle += noisyRot;

      // Keep angle in [-pi, pi) for neatness
      if(_particle_list[i].angle > PI) {
        _particle_list[i].angle -= 2.0f*PI;
      }
      else if(_particle_list[i].angle <= -PI){
        _particle_list[i].angle += 2.0f*PI;
      }

      // Now move the particle in its *local* orientation
      float cosA = cos(_particle_list[i].angle);
      float sinA = sin(_particle_list[i].angle);

      // Add the displacement
      _particle_list[i].x += (noisyX * cosA - noisyY * sinA);
      _particle_list[i].y += (noisyX * sinA + noisyY * cosA);
    }
}

/* Calculate particle posterior probabilities*/
void ParticleFilter::measure(){
  float norm_factor=0;
  float maxprob=-99;

  // Get actual sensor reading from ultrasonic
  float sensorVal = _sonar.readDist();  // in cm

  //Put next chunk in for loop:
    // compute what the distance should be, if particle position is accurate
    /*If the robot uses closest_distance to calculate posterior probabilities in a particle filter,
      the return of this function helps assess the likelihood of the particle's position given the map layout.
      
      For example, if a particle's estimated sensor reading (distance to nearest obstacle) 
      aligns with the robot's actual sensor reading, 
      this particle is likely to be close to the robot’s true position.*/
      //TODO: Fill in "..."

  for(uint8_t i=0; i<_num_particles; i++){
    float origin[2] = {
      _particle_list[i].x / 48.0f,
      _particle_list[i].y / 48.0f
    };

    float particleDist = _mp.closest_distance(origin, _particle_list[i].angle);
    // compute the probability P[measured z | robot @ x]
    //TODO: Put code under here
    float expectedDistCM = particleDist * 48.0f;
    Gaussian measureGauss(expectedDistCM, _measurement_variance * _measurement_variance);
    float prob = measureGauss.plot(sensorVal);
    /* compute the probability P[robot@x | measured]
     NOTE: This is not a probability, since we don't know P[measured z]
     Hence we normalize afterwards */
    //TODO: Put code under here
    _particle_list[i].probability *= prob;

    // Track maximum probability for resampling
    if(_particle_list[i].probability > maxprob){
      maxprob = _particle_list[i].probability;
    }
    // normalize probabilities (take P[measured z into account])
    //TODO: Put code under here
    norm_factor += _particle_list[i].probability;
  }
  //End of for loop

  //take each probability and normalize by norm_factor (in a for loop)
  //TODO: Put code under here
  if(norm_factor > 0){
    for(uint8_t i = 0; i < _num_particles; i++){
      _particle_list[i].probability /= norm_factor;
    }
  }
  else{
    for(uint8_t i =0; i < _num_particles; i++){
      _particle_list[i].probability = 1.0f / _num_particles;
    }
    maxprob = 1.0f / _num_particles;
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
  for(uint8_t i = 0; i < _num_particles; i++){
    float randVal = (float)random(0,1000) / 10000.0f;

    b += randVal * 2.0f * maxprob;
    while(b > _particle_list[index].probability){
      b -= _particle_list[index].probability;
      index = (index + 1) % _num_particles;
    }
    temp_particles[i] = _particle_list[index];
  }
  //End of for loop

  //take each temp particle probability and normalize by norm_tot
  //This should be in a for loop
  //TODO: Put code under here
  for(uint8_t i = 0; i < _num_particles; i++){
    norm_tot += temp_particles[i].probability;
  }
  if(norm_tot > 0){
    for(uint8_t i = 0; i < _num_particles; i++){
      temp_particles[i].probability /= norm_tot;
    }
  } else{
    for(uint8_t i = 0; i < _num_particles; i++){
      temp_particles[i].probability = 1.0f / _num_particles;
    }
  }
  //End of for loop

  //Set each particle in _particle_list to temp_particles
  //This should be in a for loop
  //TODO: Put code under here
  for(uint8_t i = 0; i < _num_particles; i++)
  {
    _particle_list[i] = temp_particles[i];
  }
  //End of for loop
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
  for(uint8_t i = 0; i < _num_particles; i++){
    _x_est += _particle_list[i].x * _particle_list[i].probability;
    _y_est += _particle_list[i].y * _particle_list[i].probability;
    _angle_est += _particle_list[i].angle * _particle_list[i].probability;
  }
  //End of for loop
}


/* confidence test: >90 % particles in same grid cell (48 cm) */
bool ParticleFilter::is_confident(float thr)
{
  estimate_position();
  int count = 0;
  for (uint8_t i = 0; i < _num_particles; i++)
    if (fabs(_particle_list[i].x - _x_est) < 48 &&
        fabs(_particle_list[i].y - _y_est) < 48)
      count++;
  return count >= thr * _num_particles;
}

/* decision logic for moving automatically */
MoveCommand ParticleFilter::decide_action(float frontDist)
{
    static uint8_t turnStep = 0;   // 0 = left, 1 = right, 2 = u‑turn

    if (frontDist > 24.0f) {
        return MOVE_FORWARD;       // path is clear → drive ahead
    }

    MoveCommand cmd;
    switch (turnStep) {            // pick next turn in the cycle
        case 0:  cmd = TURN_LEFT;   break;   // first wall → left
        case 1:  cmd = TURN_RIGHT;  break;   // second wall → right
        default: cmd = TURN_AROUND; break;   // third wall → 180°
    }

    turnStep = (turnStep + 1) % 3; // advance cycle 0→1→2→0→…
    return cmd;
}

/* Print particle position and probabilities to serial monitor */
void ParticleFilter::print_particles(){
  //print current iteration number, particles, and estimated location
  //Before estimating position, call the estimate_position function.
  //TODO: Put code under here
  estimate_position();

  Serial.println("Iteration: ");
  Serial.println(_iter++);
  for(uint8_t i=0; i<_num_particles; i++){
    Serial.println("P[");
    Serial.println(i);
    Serial.print("] X=");
    Serial.print(_particle_list[i].x);
    Serial.println(" Y=");
    Serial.print(_particle_list[i].y);
    Serial.println(" Angle=");
    Serial.print(_particle_list[i].angle);
    Serial.println(" Prob=");
    Serial.println(_particle_list[i].probability);
  }

  Serial.println("Estimated X = ");
  Serial.print(_x_est);
  Serial.println("Estimated Y = ");
  Serial.print(_y_est);
  Serial.println("Estimated Angle = ");
  Serial.print(_angle_est);
  Serial.println("-------------------");
}

