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

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	num_particles = 100; //number of particles
	
	default_random_engine gen;
	
	// Create normal (Gaussian) distributions for x, y, theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for(int i=0; i<num_particles; i++){
	  Particle newParticle;
	  newParticle.id = i;
	  newParticle.x = dist_x(gen);
	  newParticle.y = dist_y(gen);
	  newParticle.theta = dist_theta(gen);
	  newParticle.weight = 1.0;
        
      // Add particle to list of particles
      particles.push_back(newParticle);
      weights.push_back(newParticle.weight);
	}
	
	is_initialized = true; 
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	
	default_random_engine gen;
	
	// Create normal (Gaussian) distributions for x, y, theta
	normal_distribution<double> noise_x(0.0, std_pos[0]);
	normal_distribution<double> noise_y(0.0, std_pos[1]);
	normal_distribution<double> noise_theta(0.0, std_pos[2]);

	if (fabs(yaw_rate) < 0.0001) {
		yaw_rate = 0.0001;
	}

	for (int i=0; i<num_particles; i++){
    
        particles[i].x += (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta)) + noise_x(gen);
        particles[i].y += (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t)) + noise_y(gen);
        particles[i].theta += yaw_rate * delta_t + noise_theta(gen);
        
    }	
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	
	double var_range = std_landmark[0]*std_landmark[0];
	double var_bearing = std_landmark[1]*std_landmark[1];
	double det_std = std_landmark[0] * std_landmark[1];
	
	for(int i=0; i<num_particles; i++){
	  
	  particles[i].weight =1;
	  
	  for(int j=0; j<observations.size(); j++){
	    
		double obs_x_trans = observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta) + particles[i].x;
		double obs_y_trans = observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta) + particles[i].y;
	    double dist_min = sensor_range;
	    double dist = sensor_range;
	    double landmark_x = sensor_range;
	    double landmark_y = sensor_range;
	    
	    for(int k=0; k<map_landmarks.landmark_list.size(); k++){
	
				double dist = sqrt((obs_x_trans - map_landmarks.landmark_list[k].x_f)*(obs_x_trans - map_landmarks.landmark_list[k].x_f) 
				+ (obs_y_trans - map_landmarks.landmark_list[k].y_f)*(obs_y_trans - map_landmarks.landmark_list[k].y_f));

				if (dist < dist_min) {
					dist_min = dist;
					landmark_x = map_landmarks.landmark_list[k].x_f;
					landmark_y = map_landmarks.landmark_list[k].y_f;
				}
	    } // search map loop
	    
	    double x_diff = obs_x_trans - landmark_x;
		double y_diff = obs_y_trans - landmark_y;
		double nom = exp(-0.5*((x_diff * x_diff)/var_range + (y_diff * y_diff)/var_bearing));
		double denom = 2*M_PI*det_std;
		particles[i].weight *= nom/denom;
	    
	  } // observation loop
	  
	  weights[i] = particles[i].weight;
	  
	} // particle loop

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;

    discrete_distribution<> weights_dis(weights.begin(), weights.end());
    // initialise new particle array
    vector<Particle> newParticles;
    // resample particles
    for (int i = 0; i < num_particles; ++i)
        newParticles.push_back(particles[weights_dis(gen)]);

    particles = newParticles;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
