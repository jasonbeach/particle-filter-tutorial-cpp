#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.auxiliary.hpp"

ParticleFilterAuxiliary::ParticleFilterAuxiliary(
  double number_of_particles, const ParticleFilter::LimitsParameters& limits,
  const ParticleFilter::ProcessNoiseParameters& process_noise_params,
  const ParticleFilter::MeasurementNoiseParameters& measurement_noise_params) :
    ParticleFilter(number_of_particles, limits, process_noise_params, measurement_noise_params) {}

void ParticleFilterAuxiliary::update(double robot_forward_motion, double robot_angular_motion,
                                     const MeasurementList& measurements,
                                     const LandmarkList& landmarks) {
  // First loop: propagate characterizations and compute weights
  ParticleList tmp_particles;
  std::vector<double> tmp_likelihoods;

  for (auto particle : particles_) {  // ok to make a copy of the particle here

    // Compute characterization
    const auto mu = propagate_sample(particle, robot_forward_motion, robot_angular_motion);

    // Compute and store current particle's weight
    const auto likelihood = compute_likelihood(mu, measurements, landmarks);
    particle.weight *= likelihood;
    tmp_likelihoods.push_back(likelihood);

    // Store (notice mu will not be used later)
    tmp_particles.push_back(particle);
  }

  // Normalize particle weights
  tmp_particles = normalize_weights(tmp_particles);

  // Resample indices from propagated particles
  const auto resampled_particles = resample_multinomial(tmp_particles, tmp_particles.size());

  // Second loop: now propagate the state of all particles that survived. std::transform allows us
  // to iterate through resampled_particles and tmp_likelihoods at the same time.
  ParticleList new_samples;
  std::transform(resampled_particles.begin(), resampled_particles.end(), tmp_likelihoods.begin(),
                 std::back_inserter(new_samples), [&](const Particle& sample, double likelihood) {
                   auto propagated_sample =
                     propagate_sample(sample, robot_forward_motion, robot_angular_motion);
                   const auto wi_tmp = std::clamp(likelihood, 1e-10, 1.0);
                   propagated_sample.weight =
                     compute_likelihood(propagated_sample, measurements, landmarks) / wi_tmp;
                   return propagated_sample;
                 });
  particles_ = normalize_weights(new_samples);
}
