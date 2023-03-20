#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.nepr.hpp"

ParticleFilterNEPR::ParticleFilterNEPR(
  double num_particles, const ParticleFilter::LimitsParameters& limits,
  const ParticleFilter::ProcessNoiseParameters& process_noise_params,
  const ParticleFilter::MeasurementNoiseParameters& measurement_noise_params,
  const ResamplingAlgorithms resample_algorithm, double number_of_effective_particles_threshold) :
    ParticleFilterSIR {num_particles, limits, process_noise_params, measurement_noise_params,
                       resample_algorithm} {
  /**
   * @brief Override method that determines whether or not a step is needed for the current particle
   * filter state estimate. Resampling only occurs if the approximated number of effective particles
   * falls below the user-specified threshold. Approximate number of effective particles: 1 /
   * (sum_i^N wi^2), P_N^2 in [1].
   *
   * [1] Martino, Luca, Victor Elvira, and Francisco Louzada. "Effective sample size for importance
   * sampling based on discrepancy measures." Signal Processing 131 (2017): 386-401.
   *
   * @return true needs resampling
   * @return false does not need resampling
   */

  set_needs_resampling_function([this, number_of_effective_particles_threshold]() {
    const double sum_weights_squared = std::accumulate(
      particles().begin(), particles().end(), 0.0, [](double sum, const SimpleParticle& sample) {
        return sum += sample.weight * sample.weight;
      });

    return 1.0 / sum_weights_squared < number_of_effective_particles_threshold;
  });
}
