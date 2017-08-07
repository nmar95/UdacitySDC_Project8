//particle_filter.cpp

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
	// TODO:
    // 1. Set the number of particles.
    // 2. Initialize all particles to 1st position (based on estimates of x, y, theta and uncertainties) and weights to 1.)
	// 3. Add random Gaussian noise to each particle.

    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    // MY CODE:

    // Taken from quiz 1:

    // Give particle number a value
    num_particles = 200;
    default_random_engine gen;

    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    // Loop through each particle, initialize position, uncertainties and weights
    for (int i = 0; i < num_particles; i++) {
        // Create new particle
        Particle new_p;
        // Set Particle id
        new_p.id = i;
        // Set Particle x pose
        new_p.x = dist_x(gen);
        // Set Particle y pose
        new_p.y = dist_y(gen);
        // Set Particle theta pose
        new_p.theta = dist_theta(gen);
        // Set Particle weight to 1
        new_p.weight = 1.0;
        // Push back weight and particle values
        weights.push_back(1.0);
        particles.push_back(new_p);
    }
    // ParticleFilter is now intitialized with starting values
    is_initialized = true;

    // Initialization function is completed.
    return;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO:
    // 1. Add measurements to each particle and add random Gaussian noise.

    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.

    // MY CODE:

    default_random_engine gen;

    // Create normal disctributions for  x, y, and theta values
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    // Loop through all particles and apply predictions for each particle
    for(int i = 0; i < num_particles; ++i){
        double x_og = particles[i].x;
        double y_og = particles[i].y;
        double theta_og = particles[i].theta;
        double theta_new = theta_og + yaw_rate * delta_t;

        // Check for yaw rate equal to zero
        if(fabs(yaw_rate) > 0.00001){
            particles[i].x = x_og + velocity/yaw_rate * (sin(theta_new)-sin(theta_og));
            particles[i].y = y_og + velocity/yaw_rate * (cos(theta_og)-cos(theta_new));
            particles[i].theta = theta_new;
        }else{
            particles[i].x = x_og + velocity * sin(theta_og) * delta_t;
            particles[i].y = y_og + velocity * cos(theta_og) * delta_t;
            particles[i].theta = theta_og;
        }

        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }
    return;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO:
    // 1. Find the predicted measurement that is closest to each observed measurement
    //     and assign the observed measurement to this particular landmark.

    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.


    // MY CODE:

		// Not needed
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO:
    // 1. Update the weights of each particle using a mult-variate Gaussian distribution.

    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).

    // MY CODE:

    // Clear previous weights
		weights.clear();

    // Loop through all particles
		for(int i = 0; i < num_particles; i++){

				// Intialize variable for predicted x position
        double x_predicted = particles[i].x;
				// Intialize variable for predictedy position
				double y_predicted = particles[i].y;
        // Intialize variable for predicted theta
				double theta_predicted = particles[i].theta;

				// Initialize predicted car location vector
				std::vector<LandmarkObs> predicted;

        // Transform car observations to map coordinates assuming that the particle is the car.
        particles[i].associations.clear();
        particles[i].sense_x.clear();
        particles[i].sense_y.clear();
        double weight = 1;

				// Loop through observations
        for(int j = 0; j < observations.size(); j++){
						// Ititialize variable for observed x position
						double observed_x = observations[j].x;
						// Ititialize variable for observed y position
            double observed_y = observations[j].y;
						// Ititialize variable for observed x position on the map
						double observed_x_map = observed_x * cos(theta_predicted) - observed_y * sin(theta_predicted) + x_predicted;
            // Ititialize variable for observed y position on the map
						double observed_y_map = observed_x * sin(theta_predicted) + observed_y * cos(theta_predicted) + y_predicted;
						// Determine if the predicted positions and close enough to the observed positions
						if(pow(pow(observed_x_map - x_predicted, 2) + pow(observed_y_map - y_predicted, 2), 0.5) > sensor_range) continue;
            particles[i].sense_x.push_back(observed_x_map);
            particles[i].sense_y.push_back(observed_y_map);
            double min_range = 1000000000;
            int min_k=-1;
						// Loop through the landmarks list
						for(int k = 0; k < map_landmarks.landmark_list.size(); k++){
                // Ititialize variable for landmark x position
								double landmark_x = map_landmarks.landmark_list[k].x_f;
                // Ititialize variable for observed x position
								double landmark_y = map_landmarks.landmark_list[k].y_f;
                // Ititialize variable for difference between x position of landmark and observed x position
								double diff_x = landmark_x - observed_x_map;
                // Initialize variable for difference between y position of landmark and observed y position
								double diff_y = landmark_y - observed_y_map;
								// Initialize variable to find the range between observed position and landmark position
								double range = pow(pow(diff_x,2)+pow(diff_y,2),0.5);
								// Set min_k = k if range is less than the min_range
								if(range < min_range){
                    min_range = range;
                    min_k = k;
                }
            }
						// Initialize variable for x location of landmark
						double landmark_x = map_landmarks.landmark_list[min_k].x_f;
						// Initialize variable for y location of landmark
						double landmark_y = map_landmarks.landmark_list[min_k].y_f;
						// Pushback the landmark list id
            particles[i].associations.push_back(map_landmarks.landmark_list[min_k].id_i);
						// Update weight based on landmark position proximity
            weight = weight * exp(-0.5 * (pow((landmark_x - observed_x_map) / std_landmark[0],2) + pow((landmark_y - observed_y_map) / std_landmark[1],2))) / (2*M_PI*std_landmark[0]*std_landmark[1]);
  			}
				// Push back the updated weights
				particles[i].weight = weight;
        weights.push_back(weight);
    }
    // Update function is completed.
    return;
}

void ParticleFilter::resample() {
	// TODO:
    // 1. Resample particles with replacement with probability proportional to their weight.

    // MY CODE:

    default_random_engine gen;
    discrete_distribution<int> distribution(weights.begin(), weights.end());

		// Initialize resample particle vector
    std::vector<Particle> new_particles;

    // Clear current weights
		weights.clear();

    // Loop through and resample all particles (resample wheel)
    for(int i=0; i < num_particles; ++i){
        int new_part = distribution(gen);
        new_particles.push_back(particles[new_part]);
        weights.push_back(particles[new_part].weight);
    }

		// Set particles equal to new particles
    particles = new_particles;

    // Resample function is completed.
    return;
}


//                                              Don't Touch Beyond This Point
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
