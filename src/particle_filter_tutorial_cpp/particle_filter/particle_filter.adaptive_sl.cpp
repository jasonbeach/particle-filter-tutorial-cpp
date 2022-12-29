#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.adaptive_sl.hpp"

#include "particle_filter_tutorial_cpp/resampling/resampler.helpers.hpp"
#include "particle_filter_tutorial_cpp/resampling/resampler.hpp"

AdaptiveParticleFilterSl::AdaptiveParticleFilterSl(
  double num_particles, const LimitsParameters& limits, const ProcessNoiseParameters& process_noise,
  const MeasurementNoiseParameters& measurement_noise, const Parameters& params) :
    ParticleFilter(num_particles, limits, process_noise, measurement_noise), params_(params) {}

void AdaptiveParticleFilterSl::update(double robot_forward_motion, double robot_angular_motion,
                                      const MeasurementList& measurements,
                                      const LandmarkList& landmarks) {
  ParticleList new_particles;
  new_particles.reserve(params_.max_number_of_particles);
  double sum_likelihoods = 0.0;

  while (sum_likelihoods < params_.sum_likelihoods_threshold &&
         new_particles.size() < params_.max_number_of_particles) {
    const auto sample = draw_sample_by_weight(cumulative_sum(particles_));
    auto propagated_sample = propagate_sample(*sample, robot_forward_motion, robot_angular_motion);
    propagated_sample.weight = compute_likelihood(propagated_sample, measurements, landmarks);
    sum_likelihoods += propagated_sample.weight;
    new_particles.push_back(propagated_sample);
  }
  particles_ = normalize_weights(new_particles);
}
