#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.nepr.hpp"

ParticleFilterNEPR::ParticleFilterNEPR(
  double num_particles, const ParticleFilter::LimitsParameters& limits,
  const ParticleFilter::ProcessNoiseParameters& process_noise_params,
  const ParticleFilter::MeasurementNoiseParameters& measurement_noise_params,
  const ResamplingAlgorithms resample_algorithm, double number_of_effective_particles_threshold) :
    ParticleFilterSIR {num_particles, limits, process_noise_params, measurement_noise_params,
                       resample_algorithm},
    number_of_effective_particles_threshold_(number_of_effective_particles_threshold) {}

bool ParticleFilterNEPR::needs_resampling() const {
  const double sum_weights_squared = std::accumulate(
    particles().begin(), particles().end(), 0.0,
    [](double sum, const SimpleParticle& sample) { return sum += sample.weight * sample.weight; });

  return 1.0 / sum_weights_squared < number_of_effective_particles_threshold_;
}