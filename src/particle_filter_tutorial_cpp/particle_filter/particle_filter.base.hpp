#pragma once
#include <tuple>

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.types.hpp"

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

  ParticleFilter() = default;
  ParticleFilter(double num_particles, const LimitsParameters& limits,
                 const ProcessNoiseParameters& process_noise,
                 const MeasurementNoiseParameters& measurement_noise);

  virtual ~ParticleFilter() = default;

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

  /**
   * @brief Validate the state. State values outide allowed ranges will be corrected for assuming a
   * 'cyclic world'.
   *
   * @param particle Input particle state.
   * @return Particle Validated particle state.
   */
  Particle validate_state(const Particle& particle);

  /**
   * @brief Initialize the particle filter using the given set of particles.
   *
   * @param particles Initial particle set.
   */
  void set_particles(const ParticleList& particles);

  /**
   * @brief get particles
   *
   * @return const ParticleList&
   */
  const ParticleList& particles() const;

  /**
   * @brief Compute average state according to all weighted particles
   *
   * @return Eigen::Vector3d Average x-position, y-position and orientation
   */
  Eigen::Vector3d get_average_state() const;

  /**
   * @brief Find maximum weight in particle filter.
   *
   * @return double Maximum particle weight
   */
  double get_max_weight() const;

  /**
   * @brief Print all particles: index, state and weight.
   *
   */
  void print_particles() const;

  /**
   * @brief Normalize all particle weights.
   *
   * @param particles input particle set
   * @return ParticleList Normalized particle set
   */
  static ParticleList normalize_weights(const ParticleList& particles);

  /**
   * @brief Propagate an individual sample with a simple motion model that assumes the robot rotates
   * angular_motion rad and then moves forward_motion meters in the direction of its heading. Return
   * the propagated sample (leave input unchanged).
   *
   * @param sample Sample (unweighted particle) that must be propagated
   * @param forward_motion Forward motion in meters
   * @param angular_motion Angular motion in radians
   * @return Particle propagated sample
   */
  Particle propagate_sample(const Particle& sample, double forward_motion, double angular_motion);

  /**
   * @brief Compute likelihood p(z|sample) for a specific measurement given sample state and
   * landmarks.
   *
   * @param sample sample for which likelihood is to be computed
   * @param measurements List with measurements, for each landmark [distance_to_landmark,
   * angle_wrt_landmark], units are meters and radians
   * @param landmarks Positions (absolute) landmarks (in meters)
   * @return double
   */
  double compute_likelihood(const Particle& sample, const MeasurementList& measurements,
                            const LandmarkList& landmarks);

  /**
   * @brief Process a measurement given the measured robot displacement. Abstract method that must
   * be implemented in derived class.
   *
   * @param robot_forward_motion Measured forward robot motion in meters.
   * @param robot_angular_motion Measured angular robot motion in radians.
   * @param measurements Measurements
   * @param landmarks Landmark Positions
   */
  virtual void update(double robot_forward_motion, double robot_angular_motion,
                      const MeasurementList& measurements, const LandmarkList& landmarks) = 0;

 protected:
  LimitsParameters limits_parameters_;
  ProcessNoiseParameters process_noise_parameters_;
  MeasurementNoiseParameters measurement_noise_parameters_;
  ParticleList particles_;
  double num_particles_;
};
