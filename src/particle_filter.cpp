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

	num_particles = 500;

	default_random_engine gen;

	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];

	// Create normal distributions for x, y and theta
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	for(int i=0; i<num_particles; i++) {
        Particle particle;

        particle.id = i;
        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
        particle.weight = 1;

        // Todo: initialization of associations, sense_x and sense_y

        particles.push_back(particle);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;

	cout << "velocity: " << velocity;
	cout << "  yaw_rate: " << yaw_rate;
	cout << "  delta_t: " << delta_t << endl;

	// todo: adding distribution logic should be added. Otherwise, particle would be uniformly shifted.
	for(int i=0; i<num_particles; i++) {
//        normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
//        normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
//        normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);
        normal_distribution<double> dist_x(0.0, std_pos[0]);
        normal_distribution<double> dist_y(0.0, std_pos[1]);
        normal_distribution<double> dist_theta(0.0, std_pos[2]);

//        particles[i].x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
//        particles[i].y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
//        particles[i].theta = particles[i].theta + yaw_rate * delta_t;

        if(yaw_rate != 0.0) {
          particles[i].x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta)) + dist_x(gen);
          particles[i].y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t)) + dist_y(gen);
          particles[i].theta = particles[i].theta + yaw_rate * delta_t + dist_theta(gen);
        }
        else {
          particles[i].x = particles[i].x + velocity * delta_t + dist_x(gen);
          particles[i].y = particles[i].y + dist_y(gen);
          particles[i].theta = particles[i].theta + dist_theta(gen);
        }
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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


	// arguments: sensor_range = 50[m], sigma_landmark[2] = {0.3, 0.3}, noisy_observations = [x, y][x_sense.size()], map = [id, x, y][size]

    std::vector<double> new_weights;

	for(int i=0; i<num_particles; i++) {

	    double particle_weight = 1.0;
	    Particle particle;

        for(int j=0; j<observations.size(); j++) {

            // transform noisy_observations in the VEHICLE'S coordinate system to the MAP'S coordinate system
            LandmarkObs ob_map;
            ob_map = transformObservationCoordinateVehicleToMap(particles[i], observations[j]);

            // associate each noisy_observation with the nearest landmark
            LandmarkObs nearest_landmark;
            nearest_landmark = getNearestLandmark(ob_map, map_landmarks);

            particle.associations.push_back(nearest_landmark.id);
            particle.sense_x.push_back(ob_map.x);
            particle.sense_y.push_back(ob_map.y);
//            particles[i].associations.push_back(nearest_landmark.id);
//            particles[i].sense_x.push_back(ob_map.x);
//            particles[i].sense_y.push_back(ob_map.y);

            // calculate Multivariate-Gaussian probability of each observation
            double weight = getObservationWeight(ob_map, nearest_landmark, std_landmark);

            // calculate particle's final weight
            particle_weight *= weight;
        }

        particles[i].associations = particle.associations;
        particles[i].sense_x = particle.sense_x;
        particles[i].sense_y = particle.sense_y;
//        SetAssociations(particles[i], particle.associations, particle.sense_x, particle.sense_y);
        particles[i].weight = particle_weight;
        new_weights.push_back(particle_weight);
	}

	weights = new_weights;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// todo: get max weight
	double max_weight = *max_element(weights.begin(), weights.end());

	double beta = 0.0;
	std::vector<Particle> new_particles;

	std::mt19937 mt{ std::random_device{}() };
	std::uniform_int_distribution<int> dist_index(0, num_particles-1);
    std::uniform_real_distribution<double> dist_beta(0, 2.0 * max_weight);

	int index = dist_index(mt);

	for(int i=0; i<num_particles; i++) {
        beta += dist_beta(mt);

        while(particles[index].weight < beta) {
            beta -= particles[index].weight;
            index =  (index + 1) % num_particles;
        }

        new_particles.push_back(particles[index]);
	}

	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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

LandmarkObs ParticleFilter::transformObservationCoordinateVehicleToMap(Particle particle, LandmarkObs observation) {
    LandmarkObs ob_map;

    ob_map.x = particle.x + (cos(particle.theta) * observation.x) - (sin(particle.theta) * observation.y);
    ob_map.y = particle.y + (sin(particle.theta) * observation.x) + (cos(particle.theta) * observation.y);

    return ob_map;
}

LandmarkObs ParticleFilter::getNearestLandmark(LandmarkObs observationInMap, const Map &map_landmarks) {
    LandmarkObs nearest_landmark;
    double min_distance = INFINITY;
    double distance;

    for(int i=0; i<map_landmarks.landmark_list.size(); i++) {
        distance = dist(observationInMap.x, observationInMap.y, map_landmarks.landmark_list[i].x_f, map_landmarks.landmark_list[i].y_f);

        if(distance < min_distance) {
            nearest_landmark.id = map_landmarks.landmark_list[i].id_i;
            nearest_landmark.x  = map_landmarks.landmark_list[i].x_f;
            nearest_landmark.y  = map_landmarks.landmark_list[i].y_f;
            min_distance = distance;
        }
    }

    return nearest_landmark;
}

double ParticleFilter::getObservationWeight(LandmarkObs observationInMap, LandmarkObs landmark, double *std_landmark) {
    double gauss_norm = (1/(2 * M_PI * std_landmark[0] * std_landmark[1]));
    double exponent   = pow(observationInMap.x - landmark.x, 2)/(2 * pow(std_landmark[0], 2))
                      + pow(observationInMap.y - landmark.y, 2)/(2 * pow(std_landmark[1], 2));
    double weight = gauss_norm * exp(-exponent);

    return weight;
}

double ParticleFilter::getParticleWeight(Particle particle, double *std_landmark,
                                         const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
    double particle_weight = 1.0;

    for (int i=0; i<observations.size(); i++) {

        // transform noisy_observations in the VEHICLE'S coordinate system to the MAP'S coordinate system
        LandmarkObs ob_map;
        ob_map = transformObservationCoordinateVehicleToMap(particle, observations[i]);

        // associate each noisy_observation with the nearest landmark
        LandmarkObs nearest_landmark;
        nearest_landmark = getNearestLandmark(ob_map, map_landmarks);

        // calculate Multivariate-Gaussian probability of each observation
        double weight = getObservationWeight(ob_map, nearest_landmark, std_landmark);

        // calculate particle's final weight
        particle_weight *= weight;
    }

    return particle_weight;
}