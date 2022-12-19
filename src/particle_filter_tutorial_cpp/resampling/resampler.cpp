#include "resampler.hpp"

#include <numeric>
#include <random>

#include "resampler.helpers.hpp"

ParticleList resample_multinomial(const ParticleList& samples, size_t N) {
  // compute cumulative weight sums.  This converts each weight to a "bucket" proportional to the
  // magnitude of the weight. so when we uniformly pick a random number between 0 and 1, the larger
  // each particles weight (or bucket) is the more likely it is for the random number to fall into
  // that bucket
  const auto Q = cumulative_sum(samples);

  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_real_distribution<> dis(1e-6, 1.0);

  const double new_weight = 1.0 / static_cast<double>(N);  // new weights will be uniform
  ParticleList new_samples;
  new_samples.reserve(N);

  std::generate_n(std::back_inserter(new_samples), N, [&]() {
    const double u = dis(gen);          // Draw a random sample u
    const auto m = naive_search(Q, u);  // Naive search to element (alternative: binary search)
    return Particle {new_weight, m->state};
  });

  return new_samples;
}

ParticleList resample_residual(const ParticleList& samples, size_t N) {
  ParticleList weight_adjusted_samples;
  ParticleList replication_samples;

  // this function is not fully optimized
  const auto Nd = static_cast<double>(N);
  for (const auto& sample : samples) {
    const auto& [xm, wm] = sample;
    const auto Nm = std::floor(Nd * wm);
    weight_adjusted_samples.emplace_back(wm - Nm / Nd, xm);
    replication_samples.emplace_back(Nm, xm);
  }

  const auto new_samples_deterministic = replication(replication_samples);
  const auto Nt = static_cast<double>(new_samples_deterministic.size());

  if (Nd != Nt) {
    const auto weight_adjustment = Nd / (Nd - Nt);
    for (auto& s : weight_adjusted_samples) {
      s.weight *= weight_adjustment;
    }
  }

  const auto new_samples_stochastic =
    resample_multinomial(weight_adjusted_samples, static_cast<size_t>(Nd - Nt));

  // concatenate deterministic sample and stochastic samples
  auto weighted_new_samples = new_samples_stochastic;
  weighted_new_samples.insert(weighted_new_samples.end(), new_samples_deterministic.begin(),
                              new_samples_deterministic.end());

  const auto new_weight = 1.0 / static_cast<double>(weighted_new_samples.size());
  for (auto& s : weighted_new_samples) {
    s.weight = new_weight;
  }

  return weighted_new_samples;
}

ParticleList resample_stratified(const ParticleList& samples, size_t N) {
  const auto new_weight = 1.0 / static_cast<double>(N);
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_real_distribution<> dis(1e-10, new_weight);

  const auto Q = cumulative_sum(samples);
  ParticleList new_samples;
  new_samples.reserve(N);

  size_t n = 0;
  auto m = Q.begin();
  std::generate_n(std::back_inserter(new_samples), N, [new_weight, N, &n, &m, &Q]() {
    // Draw a random sample u0 and compute u
    const auto u0 = dis(gen);
    const auto u = u0 + static_cast<double>(n) / static_cast<double>(N);

    // u increases every loop hence we only move from left to right (once) while iterating Q
    // Get first sample for which cumulative sum is above u
    while (m->weight < u && m != Q.end()) {
      m++;  // no need to reset m, u always increases
    }
    n++;
    return Particle {new_weight, m->state};
  });
  return new_samples;
}

ParticleList resample_systematic(const ParticleList& samples, size_t N) {
  const auto new_weight = 1.0 / static_cast<double>(N);
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_real_distribution<> dis(1e-10, new_weight);

  const auto Q = cumulative_sum(samples);
  ParticleList new_samples;
  new_samples.reserve(N);

  const auto u0 = dis(gen);  // draw only one sample

  size_t n = 0;
  auto m = Q.begin();
  std::generate_n(std::back_inserter(new_samples), N, [new_weight, N, &n, &m, &Q, u0]() {
    // Compute u for current particle (deterministic given u0)
    const auto u = u0 + static_cast<double>(n) / static_cast<double>(N);

    // u increases every loop hence we only move from left to right (once) while iterating Q
    // Get first sample for which cumulative sum is above u
    while (m->weight < u && m != Q.end()) {
      m++;  // no need to reset m, u always increases
    }
    n++;
    return Particle {new_weight, m->state};
  });
  return new_samples;
}

ParticleList resample_factory(const ParticleList samples, double Nd, ResamplingAlgorithms alg) {
  const auto N = static_cast<size_t>(Nd);
  switch (alg) {
    case ResamplingAlgorithms::MULTINOMIAL: {
      return resample_multinomial(samples, N);
    }
    case ResamplingAlgorithms::RESIDUAL: {
      return resample_residual(samples, N);
    }
    case ResamplingAlgorithms::STRATIFIED: {
      return resample_stratified(samples, N);
    }
    case ResamplingAlgorithms::SYSTEMATIC: {
      return resample_stratified(samples, N);
    }
    default: {
      throw std::runtime_error("unknown resample algorithm given");
    }
  }
}