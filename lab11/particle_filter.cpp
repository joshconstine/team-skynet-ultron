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

  _x_est;
  _y_est;
  _angle_est;

  Map _mp = Map();

  //generate particles with random location and rotation using _lenOfMap for x, y, and angle.
  //All probabilities will be the same at the start.
  //TODO: fill in "..."
  for(uint8_t i=0;i<_num_particles; i++){
    _particle_list[i].x = (float)random(_lenOfMap);          // random [0, lenOfMap)
    _particle_list[i].y = (float)random(_lenOfMap);
    _particle_list[i].angle = ((float)random(628)) / 100.0;  // random angle in [0, 2π)
    _particle_list[i].probability = 1.0 / _num_particles;
  }

}

/* Propagate motion of particles */
void ParticleFilter::move_particles(float dx, float dy, float dtheta){
  for (int i = 0; i < _num_particles; i++) {
    float noise_dx = Gaussian(dx, _translation_variance);
    float noise_dy = Gaussian(dy, _translation_variance);
    float noise_theta = Gaussian(dtheta, _rotation_variance);

    _particle_list[i].angle += noise_theta;

    _particle_list[i].x += noise_dx * cos(_particle_list[i].angle);
    _particle_list[i].y += noise_dy * sin(_particle_list[i].angle);
  }
}

/* Calculate particle posterior probabilities*/
void ParticleFilter::measure(){
  float norm_factor = 0;
  float maxprob = -99;
  float measured = _sonar.readDist();

  for (int i = 0; i < _num_particles; i++) {
    float origin[2] = { _particle_list[i].x, _particle_list[i].y };
    float theta = _particle_list[i].angle;

    float expected = _mp.closest_distance(origin, theta);
    float prob = Gaussian::pdf(expected, _measurement_variance, measured);

    _particle_list[i].probability = prob;
    norm_factor += prob;

    if (prob > maxprob)  maxprob = prob;
  }

  // Normalize probabilities
  for (int i = 0; i < _num_particles; i++) {
    _particle_list[i].probability /= norm_factor;
  }

  resample(maxprob);
}

/* Resample particles */
void ParticleFilter::resample(float maxprob){
  float beta = 0.0;
  int index = random(_num_particles);
  float mw = maxprob;

  Particle temp_particles[25]; // max particles

  for (int i = 0; i < _num_particles; i++) {
    beta += (float)random(100) / 100.0 * 2.0 * mw;
    while (beta > _particle_list[index].probability) {
      beta -= _particle_list[index].probability;
      index = (index + 1) % _num_particles;
    }
    temp_particles[i] = _particle_list[index];
  }

  // Normalize and copy back
  float total = 0.0;
  for (int i = 0; i < _num_particles; i++) {
    total += temp_particles[i].probability;
  }

  for (int i = 0; i < _num_particles; i++) {
    temp_particles[i].probability /= total;
    _particle_list[i] = temp_particles[i];
  }
}

/* Print particle position and probabilities to serial monitor */
void ParticleFilter::print_particles(){
  Serial.print("Iteration: ");
  Serial.println(_iter++);

  estimate_position();

  for (int i = 0; i < _num_particles; i++) {
    Serial.print("P");
    Serial.print(i);
    Serial.print(": (");
    Serial.print(_particle_list[i].x);
    Serial.print(", ");
    Serial.print(_particle_list[i].y);
    Serial.print(") θ: ");
    Serial.print(_particle_list[i].angle);
    Serial.print("  prob: ");
    Serial.println(_particle_list[i].probability);
  }

  Serial.print("Estimated Position: (");
  Serial.print(_x_est);
  Serial.print(", ");
  Serial.print(_y_est);
  Serial.print(") θ: ");
  Serial.println(_angle_est);
}

/* Estimate the position of the robot using posterior probabilities */
void ParticleFilter::estimate_position(){
  _x_est = 0.0;
  _y_est = 0.0;
  _angle_est = 0.0;

  for (int i = 0; i < _num_particles; i++) {
    _x_est += _particle_list[i].x * _particle_list[i].probability;
    _y_est += _particle_list[i].y * _particle_list[i].probability;
    _angle_est += _particle_list[i].angle * _particle_list[i].probability;
  }
}
