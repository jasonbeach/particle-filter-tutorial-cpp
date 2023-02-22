#pragma once

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.base.hpp"

class AdaptiveParticleFilterKld : public ParticleFilter<SimpleParticle> {
 public:
  /**
   * @brief Parameters for the adaptive particle filter using Kullback-Leibler divergence (KLD)
   * sampling proposed in [1].
   *
   * [1] Fox, Dieter. "Adapting the sample size in particle filters through KLD-sampling." The
   * international Journal of robotics research 22.12 (2003): 985-1003.
   *
   */
  struct Parameters {
    Eigen::Vector3d resolutions;  ///> Resolution of 3D-histogram used to approximate the true
                                  /// posterior distribution (x, y, theta).
    double epsilon = 0.0;         ///> Maximum allowed distance (error) between true and estimated
                                  /// distribution when computing number of required particles.
    double upper_quantile = 0.0;  ///> Upper standard normal distribution quantile for (1-delta)
                                  /// where delta is theprobability. that the error on the estimated
                                  /// distribution will be less than epsilon.
    size_t min_number_of_particles = 0;  ///> Minimum number of particles that must be used.
    size_t max_number_of_particles = 0;  ///> Maximum number of particles that can be used.
  };

  /**
   * @brief Initialize the adaptive particle filter using Kullback-Leibler divergence (KLD) sampling
   *
   * @param number_of_particles: Number of particles.
   * @param limits: List with maximum and minimum values for x and y dimension: [xmin, xmax, ymin,
   * ymax].
   * @param process_noise: Process noise parameters (standard deviations): [std_forward,
   * std_angular].
   * @param measurement_noise: Measurement noise parameters (standard deviations): [std_range,
   * std_angle].
   * @param resolutions: Resolution of 3D-histogram used to approximate the true posterior
   * distribution (x, y, theta).
   * @param epsilon: Maximum allowed distance (error) between true and estimated distribution when
   * computing number of required particles.
   * @param upper_quantile: Upper standard normal distribution quantile for (1-delta) where delta is
   * the probability. that the error on the estimated distribution will be less than epsilon.
   * @param min_number_particles: Minimum number of particles that must be used.
   * @param max_number_particles: Maximum number of particles that can be used.
   */
  AdaptiveParticleFilterKld(double num_particles, const LimitsParameters& limits,
                            const ProcessNoiseParameters& process_noise,
                            const MeasurementNoiseParameters& measurement_noise,
                            const Parameters& params);

  /**
   * @brief Process a measurement given the measured robot displacement and adopting the
   * Kullback-Leibler divergence sampling method (KLD-sampling) proposed by Dieter Fox.
   * Assumption in this implementation: world starts at x=0, y=0, theta=0 (for mapping particle
   * state to histogram bin).
   *
   * @param robot_forward_motion Measured forward robot motion in meters.
   * @param robot_angular_motion Measured angular robot motion in radians.
   * @param measurements Measurements
   * @param landmarks Landmark Positions
   */
  void update(double robot_forward_motion, double robot_angular_motion,
              const MeasurementList& measurements, const LandmarkList& landmarks) override;

 private:
  /**
   * @brief Compute the number of samples needed within a particle filter when k bins in the
   * multidimensional histogram contain samples. Use Wilson-Hilferty transformation to approximate
   * the quantiles of the chi-squared distribution as proposed by Fox (2003).
   * @param k: Number of bins containing samples.
   * @param epsilon: Maxmimum allowed distance (error) between true and estimated distribution.
   * @param upper_quantile: Upper standard normal distribution quantile for (1-delta) where delta is
   * the probability that the error on the estimated distribution will be less than epsilon.
   * @return double Number of required particles.
   */
  double compute_required_number_of_particles(double k, double epsilon, double upper_quantile);
  Parameters params_;
};