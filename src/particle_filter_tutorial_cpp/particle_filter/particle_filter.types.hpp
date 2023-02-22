#pragma once

#include <random>
#include <vector>

#include "Eigen/Dense"
#include "fmt/color.h"
#include "fmt/format.h"
#include "particle_filter_tutorial_cpp/simulator/simulator.robot.hpp"
#include "particle_filter_tutorial_cpp/simulator/simulator.world.hpp"

struct SimpleParticle {
  SimpleParticle() = default;
  SimpleParticle(double weight_, const Eigen::Vector3d state_);
  // shouldn't hardcode state vector as size 3, but ok for now: x, y, theta
  Eigen::Vector3d state;
  double weight;
};

template <class ParticleType>
class ParticleList : private std::vector<ParticleType> {
 public:
  using ParticleListType = std::vector<ParticleType>;
  using ParticleListType::back;
  using ParticleListType::begin;
  using ParticleListType::cbegin;
  using ParticleListType::cend;
  using ParticleListType::clear;
  using ParticleListType::emplace_back;
  using ParticleListType::empty;
  using ParticleListType::end;
  using ParticleListType::front;
  using ParticleListType::insert;
  using ParticleListType::ParticleListType;  // pull constructors of std::vector in
  using ParticleListType::push_back;
  using ParticleListType::reserve;
  using ParticleListType::resize;
  using ParticleListType::size;
  using value_type = ParticleType;

  /**
   * @brief Compute average state according to all weighted particles
   *
   * @return Eigen::Vector3d Average x-position, y-position and orientation
   */
  Eigen::Vector3d get_average_state() const {
    // compute sum of all weights
    const auto sum_weights = std::accumulate(
      begin(), end(), 0.0, [](double sum, const ParticleType& p) { return sum + p.weight; });

    // Compute weighted average
    return std::accumulate(begin(), end(), Eigen::Vector3d {0, 0, 0},
                           [sum_weights](const Eigen::Vector3d& ave, const ParticleType& p) {
                             return Eigen::Vector3d {
                               ave.x() + p.weight / sum_weights * p.state.x(),
                               ave.y() + p.weight / sum_weights * p.state.y(),
                               ave.z() + p.weight / sum_weights * p.state.z()};
                           });
  }

  /**
   * @brief Find maximum weight in particle filter.
   *
   * @return double Maximum particle weight
   */
  double get_max_weight() const {
    return std::max_element(begin(), end(),
                            [](const auto& p1, const auto& p2) { return p1.weight < p2.weight; })
      ->weight;
  }

  /**
   * @brief Print all particles: index, state and weight.
   *
   */
  void print_particles() const {
    for (auto [i, p] : enumerate(*this)) {
      fmt::print("({}): {}\n", i, p);
    }
  }

  /**
   * @brief Normalize all particle weights.
   */
  void normalize_weights() {
    const auto num_particles = static_cast<double>(size());
    // compute sum of all weights
    const auto sum_weights = std::accumulate(
      begin(), end(), 0.0, [](double sum, const ParticleType& p) { return sum + p.weight; });

    if (const double new_weight = 1.0 / num_particles; sum_weights < 1e-15) {
      fmt::print(
        "{}: Weight normalization failed: sum of all weights is {} (weights will be "
        "reinitialized)\n",
        fmt::styled("Warning", fmt::fg(fmt::color::yellow)), sum_weights);

      std::for_each(begin(), end(), [new_weight](ParticleType& p) { p.weight = new_weight; });
      return;
    }

    std::for_each(begin(), end(), [sum_weights](ParticleType& p) { p.weight /= sum_weights; });
  }
};

// this should be in some utility header
constexpr double gaussian(double x, double mu, double sigma) {
  return exp(-(x - mu) * (x - mu) / (2.0 * sigma * sigma));
}

namespace fmt {
template <>
struct formatter<SimpleParticle> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const SimpleParticle& p, FormatContext& ctx) {
    return format_to(ctx.out(), "{:.1f},{:.1f},{:.1f} with w:{:.6f}", p.state.x(), p.state.y(),
                     p.state.z() * 180.0 / M_PI, p.weight);
  }
};

template <class ParticleType>
struct formatter<ParticleList<ParticleType> > {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const ParticleList<ParticleType>& list, FormatContext& ctx) {
    return format_to(ctx.out(), "particles:\n  {}",
                     fmt::join(list.particles().begin(), list.particles().end(), "\n  "));
  }
};

}  // namespace fmt

/**
 * @brief this is cool -- this is an enumerate function like python has. It allows something like:
 *         for (const auto [i, sample] : enumerate(samples)){
 *           // i is the "index" or enumerated value
 *           // sample is the ith element from samples container
 *         }
 *
 * It optionally takes an initial value so you can start at 1 instead of 0 or 100 or whatever
 */
template <typename T, typename TIter = decltype(std::begin(std::declval<T>())),
          typename = decltype(std::end(std::declval<T>()))>
constexpr auto enumerate(T&& iterable, size_t init_val = 0) {
  struct iterator {
    size_t i;
    TIter iter;
    bool operator!=(const iterator& other) const { return iter != other.iter; }
    void operator++() {
      ++i;
      ++iter;
    }
    auto operator*() const { return std::tie(i, *iter); }
  };
  struct iterable_wrapper {
    T iterable;
    size_t init_val = 0;
    auto begin() { return iterator {init_val, std::begin(iterable)}; }
    auto end() { return iterator {init_val, std::end(iterable)}; }
  };
  return iterable_wrapper {std::forward<T>(iterable), init_val};
}
