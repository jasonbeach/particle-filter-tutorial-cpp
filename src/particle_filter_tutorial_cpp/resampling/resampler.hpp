#pragma once
#include "particle_filter_tutorial_cpp/particle_filter/particle_filter.base.hpp"
#include "resampler.helpers.hpp"

/**
 * @brief Particles are sampled with replacement proportional to their weight and in
 * arbitrary order. This leads to a maximum variance on the number of times a particle will be
 * resampled, since any particle will be resampled between 0 and N times.
 *
 *       Computational complexity: O(N log(M)
 *
 * @param samples Samples that must be resampled.
 * @param N Number of samples that must be generated.
 * @return std::vector<ParticleType> Resampled weighted particles.
 */
template <class ParticleType>
std::vector<ParticleType> resample_multinomial(const std::vector<ParticleType>& samples, size_t N) {
  // compute cumulative weight sums.  This converts each weight to a "bucket" proportional to the
  // magnitude of the weight. so when we uniformly pick a random number between 0 and 1, the larger
  // each particles weight (or bucket) is the more likely it is for the random number to fall into
  // that bucket
  const auto Q = cumulative_sum(samples);

  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_real_distribution<> dis(1e-6, 1.0);

  const double new_weight = 1.0 / static_cast<double>(N);  // new weights will be uniform
  std::vector<ParticleType> new_samples;
  new_samples.reserve(N);

  std::generate_n(std::back_inserter(new_samples), N, [&]() {
    const double u = dis(gen);          // Draw a random sample u
    const auto m = naive_search(Q, u);  // Naive search to element (alternative: binary search)
    return ParticleType {new_weight, m->state};
  });

  return new_samples;
}

/**
 * @brief Particles should at least be present floor(wi/N) times due to first deterministic loop.
 * First Nt new samples are always the same (when running the function with the same input multiple
 * times).
 *
 *       Computational complexity: O(M) + O(N-Nt),
 *         where Nt is number of samples in first deterministic loop
 *
 * @param samples Samples that must be resampled.
 * @param N Number of samples that must be generated.
 * @return std::vector<ParticleType> Resampled weighted particles.
 */
template <class ParticleType>
std::vector<ParticleType> resample_residual(const std::vector<ParticleType>& samples, size_t N) {
  std::vector<ParticleType> weight_adjusted_samples;
  std::vector<ParticleType> replication_samples;

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

/**
 * @brief Loop over cumulative sum once hence particles should keep same order (however some
 * disappear, others are replicated).
 *
 *       Computational complexity: O(N)
 *
 * @param samples Samples that must be resampled.
 * @param N Number of samples that must be generated.
 * @return std::vector<ParticleType> Resampled weighted particles.
 */
template <class ParticleType>
std::vector<ParticleType> resample_stratified(const std::vector<ParticleType>& samples, size_t N) {
  const auto new_weight = 1.0 / static_cast<double>(N);
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_real_distribution<> dis(1e-10, new_weight);

  const auto Q = cumulative_sum(samples);
  std::vector<ParticleType> new_samples;
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

/**
 * @brief Loop over cumulative sum once hence particles should keep same order (however some
 * disappear, other are replicated). Variance on number of times a particle will be selected lower
 * than with stratified resampling.
 *
 *       Computational complexity: O(N)
 *
 * @param samples Samples that must be resampled.
 * @param N Number of samples that must be generated.
 * @return std::vector<ParticleType> Resampled weighted particles.
 */
template <class ParticleType>
std::vector<ParticleType> resample_systematic(const std::vector<ParticleType>& samples, size_t N) {
  const auto new_weight = 1.0 / static_cast<double>(N);
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_real_distribution<> dis(1e-10, new_weight);

  const auto Q = cumulative_sum(samples);
  std::vector<ParticleType> new_samples;
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

enum class ResamplingAlgorithms { MULTINOMIAL, RESIDUAL, STRATIFIED, SYSTEMATIC };

template <class ParticleType>
std::vector<ParticleType> resample_factory(const std::vector<ParticleType> samples, double Nd,
                                           ResamplingAlgorithms alg) {
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

class Mvn {
 public:
  Mvn(const Eigen::VectorXd& mu, const Eigen::MatrixXd& s) : mean {mu}, sigma {s} {}

  double pdf(const Eigen::VectorXd& x) const {
    double n = static_cast<double>(x.rows());
    double sqrt2pi = std::sqrt(2 * M_PI);
    double quadform = (x - mean).transpose() * sigma.inverse() * (x - mean);
    double norm = std::pow(sqrt2pi, -n) * std::pow(sigma.determinant(), -0.5);

    return norm * exp(-0.5 * quadform);
  }
  Eigen::VectorXd sample(unsigned int nr_iterations = 20) const {
    const auto n = mean.rows();
    // const auto nd = static_cast<double>(n);

    // Generate x from the N(0, I) distribution
    Eigen::VectorXd x(n);
    Eigen::VectorXd sum(n);
    sum.setZero();
    for (unsigned int i = 0; i < nr_iterations; i++) {
      x.setRandom();
      x = 0.5 * (x + Eigen::VectorXd::Ones(n));
      sum = sum + x;
    }
    sum = sum - (static_cast<double>(nr_iterations) / 2.0) * Eigen::VectorXd::Ones(n);
    x = sum / (std::sqrt(static_cast<double>(nr_iterations) / 12.0));

    // Find the eigen vectors of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(sigma);
    Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors().real();

    // Find the eigenvalues of the covariance matrix
    Eigen::MatrixXd eigenvalues = eigen_solver.eigenvalues().real().asDiagonal();

    // Find the transformation matrix
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(eigenvalues);
    Eigen::MatrixXd sqrt_eigenvalues = es.operatorSqrt();
    Eigen::MatrixXd Q = eigenvectors * sqrt_eigenvalues;

    return Q * x + mean;
  }
  Eigen::VectorXd mean;
  Eigen::MatrixXd sigma;
};