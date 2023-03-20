#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.sir.hpp"

ParticleFilterSIR::ParticleFilterSIR(
  double number_of_particles, const ParticleFilter::LimitsParameters& limits,
  const ParticleFilter::ProcessNoiseParameters& process_noise_params,
  const ParticleFilter::MeasurementNoiseParameters& measurement_noise_params,
  ResamplingAlgorithms resampling_algorithm) :
    ParticleFilter(number_of_particles, limits, process_noise_params, measurement_noise_params),
    resampler_(resampling_algorithm),
    needs_resampling_ {[]() { return true; }} {}

bool ParticleFilterSIR::needs_resampling() const {
  if (!needs_resampling_) {
    throw std::runtime_error {"needs resampling function not set"};
  }

  return needs_resampling_();
}

void ParticleFilterSIR::update(double robot_forward_motion, double robot_angular_motion,
                               const MeasurementList& measurements, const LandmarkList& landmarks) {
  ParticleList<SimpleParticle> new_particles;
  new_particles.reserve(particles_.size());

  std::transform(particles_.begin(), particles_.end(), std::back_inserter(new_particles),
                 [&](const SimpleParticle& par) {
                   auto propagated_state =
                     propagate_sample(par, robot_forward_motion, robot_angular_motion);
                   propagated_state.weight =
                     par.weight * compute_likelihood(propagated_state, measurements, landmarks);
                   return propagated_state;
                 });
  new_particles.normalize_weights();
  set_particles(new_particles);
  if (needs_resampling()) {
    set_particles(resample_factory(particles_, num_particles_, resampler_));
  }
}

void ParticleFilterSIR::set_needs_resampling_function(const ResampleFunction& needs_resampling) {
  needs_resampling_ = needs_resampling;
}
