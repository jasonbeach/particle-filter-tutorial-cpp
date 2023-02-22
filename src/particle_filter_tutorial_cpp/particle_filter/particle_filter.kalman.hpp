#pragma once

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.base.hpp"
#include "particle_filter_tutorial_cpp/resampling/resampler.hpp"

struct KalmanParticle : public SimpleParticle {
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

  KalmanParticle() = default;
  KalmanParticle(double weight_, const Eigen::Vector3d& state_, Eigen::Matrix3d covariance_);
  KalmanParticle(double weight_, const Eigen::Vector3d& state_);
};

using KalmanParticleList = ParticleList<KalmanParticle>;
/**
 * @brief Notes:
 * - State is (x, y, heading), where x and y are in meters and heading in radians
 * - State space assumed limited size in each dimension, world is cyclic (hence leaving at x_max
 *   means entering at x_min)
 * - Propagation and measurement models are largely hardcoded (except for standard deviations).
 */
class ParticleFilterKalman {
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
  ParticleFilterKalman() = default;
  /**
   * @brief Construct the extended Kalman particle filter. Resampling method is hardcoded hence no
   * argument.
   *
   * @param number_of_particles Number of particles
   * @param limits maximum and minimum values for x and y dimension: [xmin, xmax, ymin, ymax]
   * @param process_noise_params Process noise parameters (standard deviations): [std_forward,
   * std_angular]
   * @param measurement_noise_params Measurement noise parameters (standard deviations): [std_range,
   * std_angle].
   */
  explicit ParticleFilterKalman(double number_of_particles, const LimitsParameters& limits,
                                const ProcessNoiseParameters& process_noise_params,
                                const MeasurementNoiseParameters& measurement_noise_params);

  /**
   * @brief Initialize the particles uniformly over the world assuming a 3D state (x, y, heading).
   * No arguments are required and function always succeeds hence no return value.
   *
   */
  void initialize_particles_uniform();

  /**
   * @brief Particles are sampled with replacement proportional to their weight and in arbitrary
   * order. This leads to a maximum variance on the number of times a particle will be resampled,
   * since any particle will be resampled between 0 and N times.
   *
   * This function is reimplemented in this class since each particle now contains three elements
   * instead of two (covariance of extended Kalman filter is added).
   *
   */
  void multinomial_resampling();

  Eigen::Vector3d validate_state(const Eigen::Vector3d& state);

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
  double compute_likelihood(const Eigen::Vector3d& sample, const MeasurementList& measurements,
                            const LandmarkList& landmarks);

  /**
   * @brief Process a measurement given the measured robot displacement and resample.
   *
   * @param robot_forward_motion Measured forward robot motion in meters.
   * @param robot_angular_motion Measured angular robot motion in radians.
   * @param measurements Measurements.
   * @param landmarks Landmark positions.
   */
  void update(double robot_forward_motion, double robot_angular_motion,
              const MeasurementList& measurements, const LandmarkList& landmarks);

  const KalmanParticleList& particles() const;

 private:
  KalmanParticleList particles_;
  LimitsParameters limits_parameters_;
  ProcessNoiseParameters process_noise_parameters_;
  MeasurementNoiseParameters measurement_noise_parameters_;
  Eigen::DiagonalMatrix<double, 3> Q_;
  Eigen::DiagonalMatrix<double, 2> R_;
  double num_particles_;
};