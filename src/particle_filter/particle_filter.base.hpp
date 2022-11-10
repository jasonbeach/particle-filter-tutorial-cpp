#pragma once
#include <vector>

#include "Eigen/Dense"

/**
 * @brief Notes:
 *        - State is (x, y, heading), where x and y are in meters and heading in radians
 *        - State space assumed limited size in each dimension, world is cyclic (hence leaving at
 *          x_max means entering at x_min)
 *        - Abstract class
 *
 */
class ParticleFilter {
 public:
  struct LimitsParameters {
    double x_min = 0.;
    double x_max = 10.;
    double y_min = 0.;
    double y_max = 10.;
  };
  struct ProcessNoiseParameters {
    double std_forward = 0.;
    double std_angular = 0.;
  };
  struct MeasurementNoiseParameters {
    double std_range = 0.;
    double std_angle = 0.;
  };
  struct Particle {
    // shouldn't hardcode state vector as size 3, but ok for now: x, y, theta
    Eigen::Vector3d state;
    double weight;
  };
  ParticleFilter(double num_particles, const LimitsParameters& limits,
                 const ProcessNoiseParameters& process_noise,
                 const MeasurementNoiseParameters& measurement_noise);

  /**
   * @brief Initialize the particles uniformly over the world assuming a 3D state (x, y, heading).
   * No arguments are required and function always succeeds hence no return value.
   *
   */
  void initialize_particles_uniform();

  /**
   * @brief Initialize particle filter using a Gaussian distribution with dimension three: x, y,
   * heading. Only standard deviations can be provided hence the covariances are all assumed zero.
   * Again the state dimension is implicitly hardcoded here by assuming each vector has a length of
   * 3.
   *
   * @param mean_vector Mean of the Gaussian distribution used for initializing the particle states
   * @param std_vector Standard deviations (one for each dimension)
   */
  void initialize_particles_gaussian(const Eigen::Vector3d& mean_vector,
                                     const Eigen::Vector3d& std_vector);
 private:
  LimitsParameters limits_parameters_;
  ProcessNoiseParameters process_noise_parameters_;
  MeasurementNoiseParameters measurement_noise_parameters_;
  std::vector<Particle> particles_;
  double num_particles_;
};