#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.adaptive_kld.hpp"

#include "particle_filter_tutorial_cpp/resampling/resampler.helpers.hpp"
#include "particle_filter_tutorial_cpp/resampling/resampler.hpp"
#include "unordered_set"

// need to define a hash function so we can use an Eigen::Vector3i as a key to an unordered_set

// cheap knock off of boost hash_combine.
// https://stackoverflow.com/questions/35985960/c-why-is-boosthash-combine-the-best-way-to-combine-hash-values
template <typename T>
void hash_combine(size_t& seed, T const& v) {
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct Vector3iHash {
  size_t operator()(const Eigen::Vector3i& v) const {
    size_t seed = 0;
    hash_combine(seed, v.x());
    hash_combine(seed, v.y());
    hash_combine(seed, v.z());
    return seed;
  }
};

AdaptiveParticleFilterKld::AdaptiveParticleFilterKld(
  double num_particles, const LimitsParameters& limits, const ProcessNoiseParameters& process_noise,
  const MeasurementNoiseParameters& measurement_noise, const Parameters& params) :
    ParticleFilter(num_particles, limits, process_noise, measurement_noise), params_(params) {}

void AdaptiveParticleFilterKld::update(double robot_forward_motion, double robot_angular_motion,
                                       const MeasurementList& measurements,
                                       const LandmarkList& landmarks) {
  ParticleList new_particles;
  std::unordered_set<Eigen::Vector3i, Vector3iHash> bins_with_support;
  size_t number_of_required_particles = params_.min_number_of_particles;

  ssize_t num_particles_to_generate = number_of_required_particles - new_particles.size();
  while (num_particles_to_generate > 0) {
    fmt::print("Number of required particles: {} new_particles size: {} number to generate: {}\n",
               number_of_required_particles, new_particles.size(), num_particles_to_generate);

    std::generate_n(std::back_inserter(new_particles), num_particles_to_generate, [&]() {
      // Get sample from discrete distribution given by particle weights
      const auto random_sample = draw_sample_by_weight(particles_);

      // Propagate state of selected particle
      auto propagated_state =
        propagate_sample(*random_sample, robot_forward_motion, robot_angular_motion);

      // Compute the weight that this propagated state would get with the current measurement
      propagated_state.weight = compute_likelihood(propagated_state, measurements, landmarks);

      //  Next, we convert the discrete distribution of all new samples into a histogram. We must
      //  check if the new state (propagated_state) falls in a histogram bin with support or in an
      //  empty bin.

      //  Map state to bin indices
      const Eigen::Vector3i indices = {
        static_cast<int>(std::floor(propagated_state.state.x() / params_.resolutions.x())),
        static_cast<int>(std::floor(propagated_state.state.y() / params_.resolutions.y())),
        static_cast<int>(std::floor(propagated_state.state.z() / params_.resolutions.z()))};
      // bins_with_support only inserts if the value doesn't already exist so the container itself
      // manages whether to actually insert an element. Since it enforces unique elements, no need
      // to maintain a counter of how many unique elements get inserted. An unordered_set doesn't
      // maintain a count of many times an element was inserted--if we wanted to do that we would
      // need to switch to an std::unordered_map
      bins_with_support.insert(indices);
      return propagated_state;
    });
    const size_t number_of_bins_with_support = bins_with_support.size();
    if (number_of_bins_with_support > 1) {
      number_of_required_particles = compute_required_number_of_particles(
        number_of_bins_with_support, params_.epsilon, params_.upper_quantile);
    }
    number_of_required_particles =
      std::clamp(number_of_required_particles, params_.min_number_of_particles,
                 params_.max_number_of_particles);
    num_particles_to_generate = number_of_required_particles - new_particles.size();
  }
  particles_ = normalize_weights(new_particles);
}

size_t AdaptiveParticleFilterKld::compute_required_number_of_particles(size_t k, double epsilon,
                                                                       double upper_quantile) {
  // Helper variable (part between curly brackets in (7) in Fox paper
  double x = 1.0 - 2.0 / (9.0 * (k - 1)) + sqrt(2.0 / (9.0 * (k - 1))) * upper_quantile;
  return ceil((k - 1) / (2.0 * epsilon) * x * x * x);
}