/*
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 *
 *
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <string>
#include <random>
#include <sstream>
#include <iterator>

using std::normal_distribution;
using std::numeric_limits;
using std::vector;
using std::uniform_int_distribution;
using std::uniform_real_distribution;
using std::string;
using std::os;
using std::stringstream;
using std::ostream_iterator;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
    if (is_initialized)
    {
      return;
    }

    num_particles = 100;  // TODO: Set the number of particles

    // Random number generator
    std::default_random_engine gen;

    //Create normal distribution generator
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    //Create the num_particles and add the noise....
    for (int i = 0; i < num_particles; ++i)
    {
      // Create a particle and add the noise
      Particle p;
      p.x += dist_x(gen);
      p.y += dist_y(gen);
      p.theta += dist_theta(gen);
      p.weight = 1;

      // Add to the particles array
      particles.push_back(p);
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
   std::default_random_engine gen;

   //Measurement noise is assumed to be mean 0.
   normal_distribution<double> dist_x(0, std_pos[0]);
   normal_distribution<double> dist_y(0, std_pos[1]);
   normal_distribution<double> dist_theta(0, std_pos[2]);

   //For each particle
   for (int i = 0; i < num_particles; i++)
   {
       // Add movement to the particle

       if (fabs(yaw_rate) < 0.0001)
       {
         //If the yaw rate is very small
         particles[i].x += velocity*delta_t*cos(particles[i].theta);
         particles[i].y += velocity*delta_t*sin(particles[i].theta);
       }
       else
       {
         // If the yaw rate is significant
         particles[i].x += velocity/yaw_rate * (sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
         particles[i].y += velocity/yaw_rate * (cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
         particles[i].theta += yaw_rate*delta_t;
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
   * Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */

   for (unsigned int i = 0; i<observations.size(); i++)
   {
     //For each predicted point find closest landmarks
     double mindist = numeric_limits<double>::max();
     observations[i].id = -1;

     for (unsigned j = 0; j<predicted.size(); j++)
     {
        //calculate distance
        double pt_dist = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);

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
                                   const std::vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
/**
   * Update the weights of each particle using a mult-variate Gaussian
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

   // For each particle...
  for (int i = 0; i<num_particles; i++)
  {
     // For readability
     double x = particles[i].x;
     double y = particles[i].y;
     double theta = particles[i].theta;

 //Find landmarks in range of particle
     vector<LandmarkObs> landmarks_inRange;

     for (unsigned int k = 0; k<map_landmarks.landmark_list.size(); k++)
     {
       float landmarkX = map_landmarks.landmark_list[k].x_f;
       float landmarkY = map_landmarks.landmark_list[k].y_f;
       int id = map_landmarks.landmark_list[k].id_i;

       if (dist(x, y, landmarkX, landmarkY) < sensor_range*sensor_range)
       {
         landmarks_inRange.push_back(LandmarkObs{id, landmarkX, landmarkY});
       }
     }

    // Transform observation coordinates (measured in particle's coordinate system) to map coordinates.
    // for formula see: https://www.miniphysics.com/coordinate-transformation-under-rotation.html
    vector<LandmarkObs> transformedObs;

    for (unsigned int j=0; j<observations.size(); j++)
    {
       // Transform the observation to the Map coordinates (assuming the observation is from the particle)
       double mapX = x + (cos(theta) * observations[j].x) - (sin(theta) * observations[j].y);
       double mapY = y + (sin(theta) * observations[j].x) + (cos(theta) * observations[j].y);
       transformedObs.push_back(LandmarkObs{ observations[j].id, mapX, mapY});
    }

   // Assign Id to each observation of nearest landmark it could be
    dataAssociation(landmarks_inRange, transformedObs);

    // Update Weight

    //reset weight
    particles[i].weight = 1.0;

    //For each observation....
    for (unsigned int l = 0; l < transformedObs.size(); l++)
    {
       //Get map landmark for this observation
       unsigned int id = 0;
       while(map_landmarks.landmark_list[id].id_i != transformedObs[l].id && id < map_landmarks.landmark_list.size())
       { id++; }

       double dX = transformedObs[l].x - map_landmarks.landmark_list[id].x_f;
       double dY = transformedObs[l].y - map_landmarks.landmark_list[id].y_f;

       // Multivariate-Gaussian Probability
        double weight = (1/(2*M_PI*std_landmark[0]*std_landmark[1])) * exp(-(dX*dX/(2*std_landmark[0]*std_landmark[0]) + (dY*dY/(2*std_landmark[1]*std_landmark[1]))));
        if (weight == 0) {
          // avoid weight of zero
          particles[i].weight *= 0.0001;
        } else {
          particles[i].weight *= weight;
        }
    }
  }
}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   * Using the Resampling Wheel technique from the Particle Filtes Lesson -> Resampling Wheel
   */

   std::default_random_engine gen;
   uniform_int_distribution<int> distInt(0, num_particles - 1);
   int index = distInt(gen);
   double maxWeight = numeric_limits<double>::min();;
   vector<double> weights;
   double beta = 0.0;

   for (int i = 0; i < num_particles; i++)
   {
      maxWeight = std::max(particles[i].weight, maxWeight);
      weights.push_back(particles[i].weight);
   }

   // Create vector for the resampled particles
   vector<Particle> resampledParticles;

   //Create a random value generator between 0 and the maxWeight
   //Effectively the same as rand * maxWeight if rand is 0 << 1
   uniform_real_distribution<double> distDouble(0, maxWeight);

   for (int i = 0; i < num_particles; i++)
   {
      beta += distDouble(gen) * 2.0;
      while (beta > weights[index])
      {
        beta -= weights[index];
        index = (index + 1) % particles.size();
      }

      resampledParticles.push_back(particles[index]);
   }

   particles = resampledParticles;
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
