/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

#define NUMBER_OF_PARTICLES 500
#define EPS 0.001

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.

	// Set the number of particles
	num_particles = NUMBER_OF_PARTICLES;

	// Resize weights vector
	weights.resize(num_particles);

	// Resize particles vector
	particles.resize(num_particles);

	// Engine for later generation of particles
	default_random_engine gen;

  // Creates a normal (Gaussian) distribution for x, y and theta
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

	// Initializes particles
  for (int i = 0; i < num_particles; i++){
    particles[i].id = i;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].weight = 1.0;
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	double dist = velocity * delta_t;
	double yaw = yaw_rate * delta_t;
	double vel_yaw = velocity/yaw_rate;

	// Engine for later generation of particles
	default_random_engine gen;

	// Make distribution for adding noise
	normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

	// Different equations based on if yaw_rate is zero or not
	for (int i = 0; i < num_particles; ++i) {
		// Add measurements to particles
		if (fabs(yaw_rate) > EPS) {
			particles[i].x += vel_yaw * (sin(particles[i].theta + yaw) - sin(particles[i].theta));
			particles[i].y += vel_yaw * (cos(particles[i].theta) - cos(particles[i].theta + yaw));
			particles[i].theta += yaw;
		}
		else {
			particles[i].x += dist * cos(particles[i].theta);
			particles[i].y += dist * sin(particles[i].theta);
			// Theta will stay the same due to no yaw_rate
		}

		// Add noise to the particles
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

	// For each observation
  for (unsigned int i = 0; i < observations.size(); i++) {
		// Initialize the found map in something not possible.
    int mapId = -1;
    // Initialize min distance as a really big number.
    double minDistance = numeric_limits<double>::max();

		// For each predition.
    for (unsigned j = 0; j < predicted.size(); j++ ) {
      double xDistance = observations[i].x - predicted[j].x;
      double yDistance = observations[i].y - predicted[j].y;
      double distance = xDistance * xDistance + yDistance * yDistance;
      // If the "distance" is less than min, stored the id and update min.
      if ( distance < minDistance ) {
        minDistance = distance;
        mapId = predicted[j].id;
      }
    }
    // Update the observation identifier.
    observations[i].id = mapId;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks) {
	// Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

  for (int i = 0; i < num_particles; i++) {

    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;

    // Find landmarks in particle's range.
    double sensor_range_2 = sensor_range * sensor_range;
    vector<LandmarkObs> landmarks_inRange;
    for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      int id = map_landmarks.landmark_list[j].id_i;
      double dX = x - map_landmarks.landmark_list[j].x_f;
      double dY = y - map_landmarks.landmark_list[j].y_f;
      if ( dX*dX + dY*dY <= sensor_range_2 ) {
        landmarks_inRange.push_back(LandmarkObs{ id, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f });
      }
    }

    // Transform observation coordinates.
    vector<LandmarkObs> mapObs;
    for(unsigned int j = 0; j < observations.size(); j++) {
      double rot_x = cos(theta)*observations[j].x - sin(theta)*observations[j].y + x;
      double rot_y = sin(theta)*observations[j].x + cos(theta)*observations[j].y + y;
      mapObs.push_back(LandmarkObs{ observations[j].id, rot_x, rot_y });
    }

    // Nearest Neighbor
    dataAssociation(landmarks_inRange, mapObs);

    // Calculate weights.
    for(unsigned int j = 0; j < mapObs.size(); j++) {
      double dX, dY;
      unsigned int k = 0;
      bool found = false;
      while( !found && k < landmarks_inRange.size()) {
        if ( landmarks_inRange[k].id == mapObs[j].id) {
          found = true;
          dX = mapObs[j].x - landmarks_inRange[k].x;
          dY = mapObs[j].y - landmarks_inRange[k].y;
        }
        k++;
      }

      double weight = exp(-1.0 * (dX*dX/(2*std_landmark[0]*std_landmark[0]) + (dY*dY/(2*std_landmark[1]*std_landmark[1]))) / (2*M_PI*std_landmark[0]*std_landmark[1]));
      if (weight == 0) {
        particles[i].weight = EPS;
      } else {
        particles[i].weight = weight;
      }
    }
  }
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight.
	// You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// Engine for later generation of particles
	default_random_engine gen;

	// Get weights and max weight.
  vector<double> weights;
  double maxWeight = numeric_limits<double>::min();
  for(int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
    if ( particles[i].weight > maxWeight ) {
      maxWeight = particles[i].weight;
    }
  }

  // Creating distributions.
  uniform_real_distribution<double> distBeta(0.0, maxWeight);
  uniform_int_distribution<int> distIndx(0, num_particles - 1);

  // Resampling Wheel
  int index = distIndx(gen);
  double beta = 0.0;
  vector<Particle> resampled;
  for(int i = 0; i < num_particles; i++) {
    beta += distBeta(gen) * 2.0;
    while( beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampled.push_back(particles[index]);
  }

  particles = resampled;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y){
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best){
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best){
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best){
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
