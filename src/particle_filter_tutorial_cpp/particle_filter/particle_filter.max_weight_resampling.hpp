#pragma once

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.sir.hpp"

/**
 * @brief Notes:
 * State is (x, y, heading), where x and y are in meters and heading in radians
 * State space assumed limited size in each dimension, world is cyclic (hence leaving at x_max means
 * entering at x_min) Apply core if the reciprocal of the maximum weight drops below a specific
 * value: max weight core (MWR)
 *
 */
class ParticleFilterMWR : public ParticleFilterSIR {
 public:
  /**
   * @brief Construct a particle filter that performs resampling whenever the reciprocal of maximum
   * particle weight falls below a user specified threshold value.
   *
   * @param num_particles Number of particles
   * @param limits maximum and minimum values for x and y dimension
   * @param process_noise_params Process noise parameters (standard deviations)
   * @param measurement_noise_params Measurement noise parameters
   * @param resample_algorithm Algorithm that must be used for core
   * @param resampling_threshold Resample whenever the reciprocal of the maximum particle weight
   * falls below this value.
   */
  ParticleFilterMWR(double num_particles, const ParticleFilter::LimitsParameters& limits,
                    const ParticleFilter::ProcessNoiseParameters& process_noise_params,
                    const ParticleFilter::MeasurementNoiseParameters& measurement_noise_params,
                    const ResamplingAlgorithms resample_algorithm, double resampling_threshold);

  /**
   * @brief         Override method that determines whether or not a core step is needed for the
   * current particle filter state estimate. Resampling only occurs if the reciprocal of the maximum
   * particle weight falls below the user-specified threshold. The reciprocal of the maximum weight
   * is defined by P_N^2 in [1].
   *
   * [1] Martino, Luca, Victor Elvira, and Francisco Louzada. "Effective sample size for importance
   * sampling based on discrepancy measures." Signal Processing 131 (2017): 386-401.
   *
   * @return true needs resampling
   * @return false does not need resampling
   */
  bool needs_resampling() const override;

 private:
  double resampling_threshold_ = 0.0;
};
