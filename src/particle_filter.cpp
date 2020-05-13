/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles

  // Random number generator
  std::default_random_engine gen;

  //Create normal distribution generator
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]]);
  normal_distribution<double> dist_theta(theta, std[2]]);

  //Create the num_particles and add the noise....
  for (int i = 0; i < num_particles; i++)
  {
      // Create a particle and add the noise
      Particle p;
      p.x = x + dist_x(gen);
      p.y = y + dist_y(gen);
      p.theta = theta + dist_theta(gen);
      p.weight = 1;

      // Add to the particles array
      particles.push_back(p);
  }

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
   std::default_random_engine gen;

   normal_distribution<double> dist_x(x, std_pos[0]);
   normal_distribution<double> dist_y(y, std_pos[1]]);
   normal_distribution<double> dist_theta(theta, std_pos[2]]);

   //For each particle
   for (int i = 0; i < num_particles; i++)
   {
       // Add movement to the particle

       if (fabs(yaw_rate) < 0.0001)
       {
         //If the yaw rate is very small
         particles[i].x += velocity*dt*cos(particles[i].theta) + dist_x(gen);
         particles[i].y += velocity*dt*sin(particles[i].theta) + dist_y(gen);
         particles[i].theta += particles[i] + dist_theta(gen);
       }
       else
       {
         // If the yaw rate is significant
         particles[i].x += velocity/yaw_rate * (sin(particles[i].theta+yawrate*delta_t)-sin(particles[i].theta));
         particles[i].y += velocity/yaw_rate * (cos(particles[i].theta)-cos(particles[i].theta+yawrate*delta_t));
         particles[i].theta = theta + yaw_rate*delta_t;
       }

       // Add movement noise
       particles[i].x += dist_x(gen);
       particles[i].y += dist_y(gen);
       particles[i].theta += dist_theta(gen);
     }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */

   for (int i = 0; i<observed.size(); i++)
   {
     //For each predicted point find closest landmarks
     double mindist = numeric_limits<double>::max();
     observed[i].Id = -1;

     for (int j = 0; j<predicted.length; j++)
     {
        //calculate distance
        double pt_dist = dist(predicted[j].x, observations[i].x, predicted[j].y, observations[i].y);

        // if it is
        if (pt_dist < mindist)
        {
          observations[i].id = predicted[j].id;
          mindist = pt_dist;
        }
     }
   }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
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
   */

   float theta = -M_PI/2;  // - 90 degrees



   double max_weight = 0;

   // For each particle...
   for (int i; i<particles.length; i++)
   {
      //Find landmarks in range of particle
      vector<LandmarkObs> landmarks_inRange;

      for (int k; k<map_landmarks.size(); k++)
      {
        if (dist(particles[i].x, map_landmarks[k].x, particles[i].y, map_landmarks[k].y)) < sensor_range)
        {
          landmarks_inRange.push_back(map_landmarks[k]);
        }
      }

     // Create vector for transformed observations
     vector<LandmarkObs> transformedObs(observations);

     for (int j; j<observations.size(); j++)
     {
        // Transform the observation to the Map coordinates (assuming the observation is from the particle)
        double mapX = particles[i].x + (cos(theta) * observations[j].x) -
                              (sin(theta) * observations[j].y);

        double mapY = particles[i].y + (sin(theta) * observations[j].x) -
                (cos(theta) * obserations[j].y);

        transformedObs.push_back(LandmarkObs{ observations[j].id, mapX, mapY});
     }

     // Assign Id to each observation of nearest landmark it could be
     dataAssociation(landmarks_inRange, transformedObs)

     // Update Weight
     //For each observation....
     double Weight = 1;


     for (int k = 0; k < transformedObs.size(); k++)
     {
        //Get map landmark for this observation
        int id = 0;
        while(map_landmarks[id].id ~= transformedObs[k].id)
        { id++; }



        Weight = Weight * multiv_prob(std_landmark[0], std_landmark[1],
                transformedObs[k].x, transformedObs[k].y,
                map_landmarks[id].x, map_landmarks[id].y);
     }

     particles[i].weight = Weight;

     max_weight = max(max_weight, Weight);
   }

   //Normalise weights to between 0 and 1.
   for (int i; i<particles.length; i++)
   {
     particles[i].weight = particles[i].weight / max_weight;
   }
}

double ParticleFilter::multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y)
{
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);

  return weight;
}


void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

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

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
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
