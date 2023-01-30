#pragma once

#include <numeric>
#include <random>

#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.types.hpp"

/**
 * @brief Compute cumulative sum of a list of scalar weights
 *
 * @tparam ParticleType particle type. Type must have `state` and `weight` members as well as a two
 * argument constructor to initialize those two members
 * @param samples list with weights
 * @return std::vector<ParticleType> list containing cumulative weights, length equal to length
 * input
 */
template <typename ParticleType>
std::vector<ParticleType> cumulative_sum(const std::vector<ParticleType>& samples) {
  std::vector<ParticleType> cumulative_weights;
  cumulative_weights.reserve(samples.size());  // pre-allocate sample list

  double cum_sum = 0;
  std::transform(samples.begin(), samples.end(), std::back_inserter(cumulative_weights),
                 [&cum_sum](const auto& sample) {
                   cum_sum += sample.weight;
                   return ParticleType {cum_sum, sample.state};
                 });
  return cumulative_weights;
}

/**
 * @brief Find the element in a sample list for which element.weight < x <= next_element.weight
 * within cumulativeList[lower:upper] using a naive search method
 *
 * @param cumulative_list List of elements that increase with increasing index.
 * @param x value for which has to be checked
 * @return iterator to element
 */
template <class ParticleType>
auto naive_search(const std::vector<ParticleType>& cumulative_list, double x) {
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
/**
 * @brief Find the element in a sample list for which element.weight < x <= next_element.weight
 * within cumulativeList[lower:upper] using a binary search method
 *
 * @param cumulative_list List of elements that increase with increasing index.
 * @param x value for which has to be checked
 * @return size_t Index
 */
template <class ParticleType>
auto binary_search(const std::vector<ParticleType>& cumulative_list, double x) {
  if (cumulative_list.empty()) {
    throw std::runtime_error("provided list is empty");
  }
  if (x > cumulative_list.back().weight) {
    throw std::runtime_error("x is outside of cumulative list range");
  }

  return std::lower_bound(cumulative_list.begin(), cumulative_list.end(), x,
                          [](const ParticleType& sample, double y) { return y < sample.weight; });
}

template <class ParticleType>
auto draw_sample_by_weight(const std::vector<ParticleType>& cumulative_list) {
  static std::random_device rd;
  static std::mt19937_64 gen(rd());

  if (cumulative_list.empty()) {
    throw std::runtime_error("[draw_sample_by_weight] provided particle list is empty");
  }

  std::uniform_real_distribution<> dist(1e-6, cumulative_list.back().weight);

  double u = dist(gen);
  return (naive_search(cumulative_list, u));
}

/**
 * @brief simple rounding function to robustly convert some kind of floating point number to int
 * with a guarentee that floating point precision won't be an issue.  i.e. if we want 15 but the
 * number given is 14.999999999999999 a simple static_cast will return 14. Probably being overly
 * pedantic...and this probably isn't needed but it was easy. And yes I should check T is actually a
 * floating point type.
 *
 * @param x number to convert
 * @return ssize_t converted integer type
 */
template <typename T>
ssize_t roundi(T x) {
  return static_cast<ssize_t>(x + 0.5);
}

/**
 * @brief Deterministically replicate samples.
 *
 * @param samples A list of sample whose weight represents the number of times it needs to be
 * replicated.
 * @return ParticleList of replicated samples with uninitialized weights
 */
template <class ParticleType>
auto replication(const std::vector<ParticleType>& samples) {
  std::vector<ParticleType> replicated_samples;

  for (const auto& sample : samples) {
    ssize_t Nk = roundi(sample.weight);  // attempt at robustly converting float to int
    std::generate_n(std::back_inserter(replicated_samples), Nk, [&sample]() {
      return ParticleType {0.0, sample.state};
    });
  }

  return replicated_samples;
}
