#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.sir.hpp"

ParticleFilterSIR::ParticleFilterSIR(
  double number_of_particles, const ParticleFilter::LimitsParameters& limits,
  const ParticleFilter::ProcessNoiseParameters& process_noise_params,
  const ParticleFilter::MeasurementNoiseParameters& measurement_noise_params,
  ResamplingAlgorithms resampling_algorithm) :
    ParticleFilter(number_of_particles, limits, process_noise_params, measurement_noise_params),
    resampler_(resampling_algorithm) {}

bool ParticleFilterSIR::needs_resampling() const { return true; }

void ParticleFilterSIR::update(double robot_forward_motion, double robot_angular_motion,
                               const MeasurementList& measurements, const LandmarkList& landmarks) {
  ParticleList new_particles;
  new_particles.reserve(particles().size());

  std::transform(particles_.begin(), particles_.end(), std::back_inserter(new_particles),
                 [&](const Particle& par) {
                   auto propagated_state =
                     propagate_sample(par, robot_forward_motion, robot_angular_motion);
                   propagated_state.weight =
                     par.weight * compute_likelihood(propagated_state, measurements, landmarks);
                   return propagated_state;
                 });
  set_particles(normalize_weights(new_particles));
  if (needs_resampling()) {
    set_particles(resample_factory(particles_, num_particles_, resampler_));
  }
}
