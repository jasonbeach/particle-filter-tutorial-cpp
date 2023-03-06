#include <map>

#include "fmt/core.h"
#include "particle_filter_tutorial_cpp/resampling/resampler.hpp"

struct TestParticle {
  TestParticle() = default;
  TestParticle(double _weight, int _state) : state(_state), weight(_weight) {}

  int state = 0;
  double weight = 0.0;
};

namespace fmt {
template <>
struct formatter<TestParticle> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const TestParticle& p, FormatContext& ctx) const {
    return format_to(ctx.out(), "state: {} with w:{:.6f}", p.state, p.weight);
  }
};
}  // namespace fmt

/**
 * @brief In this program a particle set will be generated and different resampling algorithms will
 * be used to resample (with replacement) from this particle set. The probability of selecting a
 * particle is proportional to its weight.
 *
 * In the end the number of times each particle has been selected (this should be the same for all
 * resampling algorithms) is printed. Furthermore, the standard deviations are shown (the variance
 * in the number of times each particle has been selected. This is a measure of the predictability
 * of the resampling algorithm. In other words, the diversity in sample set when performing
 * resampling multiple times on exactly the same particle set.
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char* argv[]) {
  fmt::print("Compare four resampling algorithms.\n");

  const size_t number_of_particles = 5;

  // create a map to store result counts in.

  ParticleList<TestParticle> samples;
  std::generate_n(std::back_inserter(samples), number_of_particles, []() {
    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    static std::uniform_real_distribution<> dis(0.0, 1.0);
    static int ctr = 1;
    return TestParticle {dis(gen), ctr++};
  });

  fmt::print("we have {} samples.\n", samples.size());
  // normalize weights
  samples.normalize_weights();

  const std::vector<ResamplingAlgorithms> methods {
    ResamplingAlgorithms::MULTINOMIAL, ResamplingAlgorithms::RESIDUAL,
    ResamplingAlgorithms::STRATIFIED, ResamplingAlgorithms::SYSTEMATIC};

  const size_t num_steps = 100'000;

  Eigen::ArrayXXd all_results;
  all_results.resize(number_of_particles, num_steps);

  for (const auto& method : methods) {
    fmt::print("Testing {} resampling\n", resampling_algorithms_str(method));

    for (auto i = 0; i < num_steps; ++i) {
      const auto weighted_samples = resample_factory(samples, number_of_particles, method);
      for (int j = 0; j < number_of_particles; ++j) {
        all_results(j, i) = std::count_if(weighted_samples.begin(), weighted_samples.end(),
                                          [j](const TestParticle& t) { return t.state == j + 1; });
      }
    }

    std::vector<double> means;
    std::transform(all_results.rowwise().begin(), all_results.rowwise().end(),
                   std::back_inserter(means), [](const auto& row) -> double { return row.mean(); });
    fmt::print("mean # occurrences: [{}]\n", fmt::join(means, ", "));

    std::vector<double> stds;
    std::transform(all_results.rowwise().begin(), all_results.rowwise().end(),
                   std::back_inserter(stds), [](const auto& row) -> double {
                     return std::sqrt((row - row.mean()).square().sum() / (row.size() - 1));
                   });

    fmt::print("std  # occurrences: [{}]\n", fmt::join(stds, ", "));
  }
}
