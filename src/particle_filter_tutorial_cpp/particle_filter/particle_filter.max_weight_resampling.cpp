#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.max_weight_resampling.hpp"

ParticleFilterMWR::ParticleFilterMWR(
  double num_particles, const ParticleFilter::LimitsParameters& limits,
  const ParticleFilter::ProcessNoiseParameters& process_noise_params,
  const ParticleFilter::MeasurementNoiseParameters& measurement_noise_params,
  const ResamplingAlgorithms resample_algorithm, double resampling_threshold) :
    ParticleFilterSIR {num_particles, limits, process_noise_params, measurement_noise_params,
                       resample_algorithm},
    resampling_threshold_(resampling_threshold) {}

bool ParticleFilterMWR::needs_resampling() const {
  const auto max_element =
    std::max_element(particles().begin(), particles().end(),
                     [](const SimpleParticle& sample1, const SimpleParticle& sample2) {
                       return sample1.weight < sample2.weight;
                     });

  return 1.0 / max_element->weight < resampling_threshold_;
}