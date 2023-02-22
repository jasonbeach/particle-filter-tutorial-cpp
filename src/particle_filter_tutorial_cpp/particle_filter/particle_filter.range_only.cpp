#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.range_only.hpp"

ParticleFilterRangeOnly::ParticleFilterRangeOnly(
  double num_particles, const ParticleFilter::LimitsParameters& limits,
  const ParticleFilter::ProcessNoiseParameters& process_noise,
  const ParticleFilter::MeasurementNoiseParameters& measurement_noise,
  ResamplingAlgorithms resampling_algorithm) :
    ParticleFilter(num_particles, limits, process_noise, measurement_noise),
    resampler_ {resampling_algorithm} {}

bool ParticleFilterRangeOnly::needs_resampling() const { return true; }

double ParticleFilterRangeOnly::compute_likelihood(const SimpleParticle& sample,
                                                   const MeasurementList& measurements,
                                                   const LandmarkList& landmarks) {
  // Initialize measurement likelihood
  auto likelihood_sample = 1.0;

  // Loop over all landmarks for current particle
  // TODO: change to std::transform
  for (auto [i, lm] : enumerate(landmarks)) {
    //  Compute expected measurement assuming the current particle state
    const auto dx = sample.state.x() - lm.x();
    const auto dy = sample.state.y() - lm.y();
    const auto expected_distance = sqrt(dx * dx + dy * dy);

    // Map difference true and expected distance measurement to probability
    const auto p_z_given_x_distance =
      gaussian(expected_distance, measurements[i].x(), measurement_noise_parameters_.std_range);

    // Incorporate likelihoods current landmark
    likelihood_sample *= p_z_given_x_distance;
  }

  return likelihood_sample;
}

void ParticleFilterRangeOnly::update(double robot_forward_motion, double robot_angular_motion,
                                     const MeasurementList& measurements,
                                     const LandmarkList& landmarks) {
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
