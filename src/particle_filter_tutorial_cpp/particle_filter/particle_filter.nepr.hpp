#pragma once

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.sir.hpp"

/**
 * @brief Notes:
 * State is (x, y, heading), where x and y are in meters and heading in radians
 * State space assumed limited size in each dimension, world is cyclic (hence leaving at x_max means
 * entering at x_min) Apply approximate number of effective particles core (NEPR)
 *
 */
class ParticleFilterNEPR : public ParticleFilterSIR {
 public:
  /**
   * @brief Construct a particle filter that performs resampling whenever the approximated number of
   * effective particles falls below a user specified threshold value.
   *
   * @param num_particles Number of particles
   * @param limits maximum and minimum values for x and y dimension
   * @param process_noise_params Process noise parameters (standard deviations)
   * @param measurement_noise_params Measurement noise parameters
   * @param resample_algorithm Algorithm that must be used for core
   * @param number_of_effective_particles_threshold Resample whenever approximate number of
   * effective particles falls below this value.
   */
  ParticleFilterNEPR(double num_particles, const ParticleFilter::LimitsParameters& limits,
                     const ParticleFilter::ProcessNoiseParameters& process_noise_params,
                     const ParticleFilter::MeasurementNoiseParameters& measurement_noise_params,
                     const ResamplingAlgorithms resample_algorithm,
                     double number_of_effective_particles_threshold);
};
