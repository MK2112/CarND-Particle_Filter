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
#include "helper_functions.h"

using namespace std;

// Needed for sampling from normal distributions
default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
    /**
     * DONE: Set the number of particles. Initialize all particles to
     *   first position (based on estimates of x, y, theta and their uncertainties
     *   from GPS) and all weights to 1.
     * DONE: Add random Gaussian noise to each particle.
     * NOTE: Consult particle_filter.h for more information about this method
     *   (and others in this file).
     */
    num_particles = 275;

    // Setup normal (Gaussian) distributions for x, y and theta
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    for (int i = 0; i < num_particles; ++i) {
        Particle p;

        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0;

        particles.push_back(p);
        weights.push_back(p.weight);
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
    /**
     * DONE: Add measurements to each particle and add random Gaussian noise.
     * NOTE: When adding noise you may find std::normal_distribution
     *   and std::default_random_engine useful.
     *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
     *  http://www.cplusplus.com/reference/random/default_random_engine/
     */

    // Setup normal (Gaussian) distributions for x, y and theta predictions
    // Defined here (and like this) for performance reasons
    normal_distribution<double> dist_x(0.0, std_pos[0]);
    normal_distribution<double> dist_y(0.0, std_pos[1]);
    normal_distribution<double> dist_theta(0.0, std_pos[2]);

    for (int i = 0; i < num_particles; ++i) {
        double x_p, y_p, theta_p;

        // Avoiding zero division
        if (yaw_rate == 0.0) {
            x_p = particles[i].x + velocity*delta_t*cos(particles[i].theta);
            y_p = particles[i].y + velocity*delta_t*sin(particles[i].theta);
        } else {
            x_p = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
            y_p = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
        }

        theta_p = particles[i].theta + yaw_rate * delta_t;

        particles[i].x = x_p + dist_x(gen);
        particles[i].y = y_p + dist_y(gen);
        particles[i].theta = theta_p + dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
    /**
     * DONE: Find the predicted measurement that is closest to each
     *   observed measurement and assign the observed measurement to this
     *   particular landmark.
     * NOTE: this method will NOT be called by the grading code. But you will
     *   probably find it useful to implement this method and use it as a helper
     *   during the updateWeights phase.
     */

    for (int i = 0; i < observations.size(); ++i) {
        double distance;
        double min_distance = -1.0;
        int min_obs_id = -1;

        for (int j = 0; j < predicted.size(); ++j) {
            // sqrt would be nice, but performance is nicer
            distance = pow(predicted[j].x - observations[i].x, 2.0) + pow(predicted[j].y - observations[i].y, 2.0);
            if (distance < min_distance || min_distance < 0.0) {
                min_distance = distance;
                min_obs_id = predicted[j].id;
            }
        }
        observations[i].id = min_obs_id;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
    /**
     * DONE: Update the weights of each particle using a mult-variate Gaussian
     *   distribution. You can read more about this distribution here:
     *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
     * NOTE: The observations are given in the VEHICLE'S coordinate system.
     *   Your particles are located according to the MAP'S coordinate system.
     *   You will need to transform between the two systems. Keep in mind that
     *   this transformation requires both rotation AND translation (but no scaling).
     *   The following is a good resource for the theory:
     *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
     *   and the following is a good resource for the actual equation to implement
     *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
     **/

    // used later for normalizing particle weights
    double weight_sum = 0.0;

    for (int i = 0; i < num_particles; ++i) {

        // Transformed observations will be stored here
        vector<LandmarkObs> observations_transformed;

        // Per Particle: Transform observations from local coordinate space to global coordinate space
        for (int j = 0; j < observations.size(); ++j) {
            observations_transformed.push_back(LandmarkObs{j,
                                                           particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y),
                                                           particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y)
            });
        }

        // Storage vector for landmarks within sensor range of this particle
        vector<LandmarkObs> predicted_landmarks;

        // Create vector of landmarks within sensor range of this particle
        for (int k = 0; k < map_landmarks.landmark_list.size(); ++k) {
            Map::single_landmark_s map_landmark = map_landmarks.landmark_list[k];
            if ((sensor_range >= fabs((particles[i].x - map_landmark.x_f)))
                && (sensor_range >= fabs((particles[i].y - map_landmark.y_f)))) {
                predicted_landmarks.push_back(LandmarkObs {map_landmark.id_i,
                                                           map_landmark.x_f,
                                                           map_landmark.y_f});
            }
        }

        // Associate transformed observations with predicted landmarks within range
        dataAssociation(predicted_landmarks, observations_transformed);

        // Setting Particle weight to 1 for easier multiplication later on
        particles[i].weight = 1.0;

        double normalizing_quotient = (M_PI * std_landmark[0] * std_landmark[1] * 2.0);

        // Calculate current particle's weight
        for (int l = 0; l < observations_transformed.size(); ++l) {
            double obs_transformed_x = observations_transformed[l].x;
            double obs_transformed_y = observations_transformed[l].y;
            double obs_transformed_id = observations_transformed[l].id;

            for (int m = 0; m < predicted_landmarks.size(); ++m) {
                double landmark_predicted_x = predicted_landmarks[m].x;
                double landmark_predicted_y = predicted_landmarks[m].y;
                double landmark_predicted_id = predicted_landmarks[m].id;

                if (obs_transformed_id == landmark_predicted_id) {
                    particles[i].weight *= (1.0 / normalizing_quotient) * exp(-1.0 * ((pow((obs_transformed_x - landmark_predicted_x), 2) / (2.0 * pow(std_landmark[0], 2))) + (pow((obs_transformed_y - landmark_predicted_y), 2) / (2.0 * pow(std_landmark[1], 2)))));
                }
            }
        }

        weight_sum += particles[i].weight;
    }

    // All particles' weights get normalized
    for (int n = 0; n < particles.size(); ++n) {
        particles[n].weight /= weight_sum;
        weights[n] = particles[n].weight;
    }
}


void ParticleFilter::resample() {
    /**
     * DONE: Resample particles with replacement with probability proportional to their weight.
     * NOTE: You may find std::discrete_distribution helpful here.
     *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
     */

    vector<Particle> resampled;

    // Generate particle index at random
    // (uniform_int_distribution seemed more fitting than discrete_distribution, I found it here: http://www.cplusplus.com/reference/random/)
    uniform_int_distribution<int> uniform_index(0, num_particles - 1);
    int weight_index = uniform_index(gen);

    double beta = 0.0;
    double mw = 0.0;

    for (int i = 0; i < num_particles; ++i) {
        if (weights[i] > mw) {
            mw = weights[i];
        }
    }

    for (int i = 0; i < num_particles; ++i) {
        // beta can get randomly increased by up to twice the highest weight
        // (uniform_real_distribution seemed more fitting than discrete_distribution, I found it here: http://www.cplusplus.com/reference/random/uniform_real_distribution/)
        uniform_real_distribution<double> uniform_weight(0.0, *max_element(weights.begin(), weights.end()) * 2);
        beta += uniform_weight(gen);

        // While this random beta is larger than the current weight: reduce beta by it
        while (beta > weights[weight_index]) {
            beta -= weights[weight_index];
            // Modulo avoids: index > N-1 ('bounces' back to beginning)
            weight_index = (weight_index + 1) % num_particles;
        }

        resampled.push_back(particles[weight_index]);
    }

    particles = resampled;
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
    // particle: the particle to which assign each listed association,
    //   and association's (x,y) world coordinates mapping
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
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
    vector<double> v;

    if (coord == "X") {
        v = best.sense_x;
    } else {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}