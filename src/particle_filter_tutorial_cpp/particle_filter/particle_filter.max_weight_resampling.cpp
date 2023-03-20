#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.max_weight_resampling.hpp"

ParticleFilterMWR::ParticleFilterMWR(
  double num_particles, const ParticleFilter::LimitsParameters& limits,
  const ParticleFilter::ProcessNoiseParameters& process_noise_params,
  const ParticleFilter::MeasurementNoiseParameters& measurement_noise_params,
  const ResamplingAlgorithms resample_algorithm, double resampling_threshold) :
    ParticleFilterSIR {num_particles, limits, process_noise_params, measurement_noise_params,
                       resample_algorithm} {
  /**
   * Resampling only occurs if the reciprocal of the maximum particle weight falls below the
   * user-specified threshold. The reciprocal of the maximum weight is defined by P_N^2 in [1].
   *
   * [1] Martino, Luca, Victor Elvira, and Francisco Louzada. "Effective sample size for importance
   * sampling based on discrepancy measures." Signal Processing 131 (2017): 386-401.
   *
   * @return true needs resampling
   * @return false does not need resampling
   */
  set_needs_resampling_function([this, resampling_threshold]() {
    const auto max_element =
      std::max_element(particles().begin(), particles().end(),
                       [](const SimpleParticle& sample1, const SimpleParticle& sample2) {
                         return sample1.weight < sample2.weight;
                       });

    return 1.0 / max_element->weight < resampling_threshold;
  });
}
