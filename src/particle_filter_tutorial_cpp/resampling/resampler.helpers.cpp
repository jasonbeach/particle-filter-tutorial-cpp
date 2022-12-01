#include "particle_filter_tutorial_cpp/resampling/resampler.helpers.hpp"

#include <numeric>
#include <random>

ParticleList cumulative_sum(const ParticleList& samples) {
  ParticleList cumulative_weights;
  cumulative_weights.reserve(samples.size());  // pre-allocate sample list

  double cum_sum = 0;
  std::transform(samples.begin(), samples.end(), std::back_inserter(cumulative_weights),
                 [&cum_sum](const auto& sample) {
                   cum_sum += sample.weight;
                   return Particle {cum_sum, sample.state};
                 });
  return cumulative_weights;
}

ParticleList::const_iterator naive_search(const ParticleList& cumulative_list, double x) {
  if (cumulative_list.empty()) {
    throw std::runtime_error("provided list is empty");
  }
  if (x > cumulative_list.back().weight) {
    throw std::runtime_error("x is outside of cumulative list range");
  }

  auto m = cumulative_list.begin();
  while (m->weight < x) {
    m++;
  }
  return m;
}

ParticleList::const_iterator binary_search(const ParticleList& cumulative_list, double x) {
  if (cumulative_list.empty()) {
    throw std::runtime_error("provided list is empty");
  }
  if (x > cumulative_list.back().weight) {
    throw std::runtime_error("x is outside of cumulative list range");
  }

  return std::lower_bound(cumulative_list.begin(), cumulative_list.end(), x,
                          [](const Particle& sample, double y) { return y < sample.weight; });
}

ssize_t roundi(double x) { return static_cast<ssize_t>(x + 0.5); }

ParticleList replication(const ParticleList& samples) {
  ParticleList replicated_samples;

  for (const auto& sample : samples) {
    ssize_t Nk = roundi(sample.weight);  // attempt at robustly converting float to int
    std::generate_n(std::back_inserter(replicated_samples), Nk, [&sample]() {
      return Particle {0.0, sample.state};
    });
  }

  return replicated_samples;
}
