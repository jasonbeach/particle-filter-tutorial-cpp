#pragma once

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.base.hpp"

class AdaptiveParticleFilterSl : public ParticleFilter {
 public:
  /**
   * @brief Parameters for the adaptive particle filter using sum of likelihoods sampling proposed
   * explained in [1,2].
   *
   * [1] Straka, Ondrej, and Miroslav Simandl. "A survey of sample size adaptation techniques for
   * particle filters." IFAC Proceedings Volumes 42.10 (2009): 1358-1363. [2] Koller, Daphne, and
   * Raya Fratkina. "Using Learning for Approximation in Stochastic Processes." ICML. 1998.
   *
   */
  struct Parameters {
    double sum_likelihoods_threshold = 0.0;  ///> Minimum sum of all measurement likelihoods after
                                             ///> update step.
    size_t max_number_of_particles = 0;      ///> Maximum number of particles that can be used.
  };

  /**
   * @brief Initialize the adaptive particle filter using sum of likelihoods sampling
   *
   * @param number_of_particles: Number of particles.
   * @param limits: List with maximum and minimum values for x and y dimension: [xmin, xmax, ymin,
   * ymax].
   * @param process_noise: Process noise parameters (standard deviations): [std_forward,
   * std_angular].
   * @param measurement_noise: Measurement noise parameters (standard deviations): [std_range,
   * std_angle].
   * @param sum_likelihoods_threshold: Minimum sum of all measurement likelihoods after update step.
   * @param max_number_particles: Maximum number of particles that can be used.
   */
  AdaptiveParticleFilterSl(double num_particles, const LimitsParameters& limits,
                           const ProcessNoiseParameters& process_noise,
                           const MeasurementNoiseParameters& measurement_noise,
                           const Parameters& params);

  /**
   * @brief Process a measurement given the measured robot displacement. Continue core as long as
   * the sum of all sampled particles is below a predefined threshold and the number of particles
   * does not exceed the predefined maximum number of particles. The minimum number of particles is
   * equal to the threhold since a measurement likelihoods cannot exceed one by definition.
   *
   * @param robot_forward_motion Measured forward robot motion in meters.
   * @param robot_angular_motion Measured angular robot motion in radians.
   * @param measurements Measurements
   * @param landmarks Landmark Positions
   */
  void update(double robot_forward_motion, double robot_angular_motion,
              const MeasurementList& measurements, const LandmarkList& landmarks) override;

 private:
  Parameters params_;
};