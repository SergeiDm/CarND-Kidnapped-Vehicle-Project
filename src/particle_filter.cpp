#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // Random number engine
	default_random_engine gen;

	// Create a normal (Gaussian) distribution
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	// Initialize number of particles
  num_particles = 100;

	// Generate particles
	for (int i = 0; i < num_particles; ++i) {
		 Particle new_point;
		 new_point.id = i;
		 new_point.x = dist_x(gen);
		 new_point.y = dist_y(gen);
		 new_point.theta = dist_theta(gen);
		 new_point.weight = 1;
		 particles.push_back(new_point);
	}

	// Set flag particles were initialized
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  // Random number engine
	default_random_engine gen;
  // Create a normal distribution
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  // Update particles locations
	for (int i = 0; i < num_particles; ++i) {
    if (yaw_rate==0) {
      particles[i].x += velocity * delta_t * cos(particles[i].theta) + dist_x(gen);
      particles[i].y += velocity * delta_t * sin(particles[i].theta) + dist_y(gen);
    } else {
      double psi = particles[i].theta + yaw_rate * delta_t;
		  particles[i].x += (velocity/yaw_rate) * (sin(psi)- sin(particles[i].theta)) + dist_x(gen);
      particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta) - cos(psi)) + dist_y(gen);
		  particles[i].theta = psi + dist_theta(gen);
    }
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {

  // Calculate some elements for using in defining particle's weights
  double denom = 1/(2*M_PI * std_landmark[0] * std_landmark[1]);
  double std0 = 2*std_landmark[0]*std_landmark[0];
  double std1 = 2*std_landmark[1]*std_landmark[1];

  // Update weights for every particle
	for (int i = 0; i < num_particles; ++i) {
	  particles[i].weight = 1;

	  // Transformation observation coordinates into map coordinate system
	  for (int j = 0; j < observations.size(); ++j) {

      double x = observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta) + particles[i].x;
      double y = observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta) + particles[i].y;

      // Associate each transformed observation with a land mark identifier by using Euclidean distance
      double dist_x, dist_y, dist;
      int min_dist_id;
      double min_dist = 1E+10;

      for (int k=0; k < map_landmarks.landmark_list.size(); ++k) {
        // Take map landmarks within sensor_range
        if ((map_landmarks.landmark_list[k].x_f  <= (particles[i].x + sensor_range)) && (map_landmarks.landmark_list[k].y_f <= (particles[i].y + sensor_range))) {
          dist_x = x - map_landmarks.landmark_list[k].x_f;
          dist_y = y - map_landmarks.landmark_list[k].y_f;
          dist = sqrt(dist_x*dist_x + dist_y*dist_y);
          if (dist < min_dist) {
            min_dist = dist;
            min_dist_id = k;
          }
        }
      }

      // Calculate particle's weights
      dist_x = x - map_landmarks.landmark_list[min_dist_id].x_f;
      dist_y = y - map_landmarks.landmark_list[min_dist_id].y_f;
      double numer = exp(-1*(dist_x*dist_x/std0+ dist_y*dist_y/std1));
      particles[i].weight *= numer/denom;
	  }
	}

}

void ParticleFilter::resample() {

  // Create vector of particle's weights
  vector<double> weights;
  for (int i = 0; i < num_particles; ++i) {
    weights.push_back(particles[i].weight);
  }

  // Store particle's information
  vector<Particle> particles_old;
  particles_old = particles;
  particles.clear();

  // Random number engine
  default_random_engine gen;

  // Produce random integers,
  // where the probability of each individual integer i is defined as the weight divided by the sum of all weights
  discrete_distribution<int> distribution(weights.begin(), weights.end());

  for (int i = 0; i < num_particles; ++i){
    int number = distribution(gen);
    particles.push_back(particles_old[number]);
  }

  weights.clear();

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();

}
