#pragma once

#include <vector>

#include "Eigen/Dense"
#include "fmt/core.h"
#include "particle_filter_tutorial_cpp/simulator/simulator.robot.hpp"
#include "particle_filter_tutorial_cpp/simulator/simulator.world.hpp"

struct Particle {
  Particle() = default;
  Particle(double weight_, const Eigen::Vector3d state_);
  // shouldn't hardcode state vector as size 3, but ok for now: x, y, theta
  Eigen::Vector3d state;
  double weight;
};
using ParticleList = std::vector<Particle>;

namespace fmt {
template <>
struct formatter<Particle> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const Particle& p, FormatContext& ctx) {
    return format_to(ctx.out(), "{:.1f},{:.1f},{:.1f} with w:{:.6f}", p.state.x(), p.state.y(),
                     p.state.z() * 180.0 / M_PI, p.weight);
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
